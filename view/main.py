# ============================================================================
# view/main.py -- OpenMV 颜色识别 → STM32 视觉帧
#
# 帧格式（7 字节，对应 control/Arm6_Unified/Core/Src/vision_frame.c）：
#     0x2C 0x12 <color> <cx> <cy> <depth> 0x5B
#   color : 'B' 蓝 / 'Y' 黄 / 'G' 绿 / 'R' 红 / 'A' 无目标（心跳）
#   cx,cy : 色块中心像素（QQVGA 160x120，单字节足够）
#   depth : 目标到相机的距离，单位 mm，0..255
#   SEND_DEPTH = False 时退化为 6 字节帧，STM32 用默认深度 ARM_DEFAULT_DEPTH_MM
#
# 本脚本每帧必定发一帧（ALWAYS_SEND=True）：没有目标时发 'A'，
# 保证链路上始终有数据，便于用串口助手或 $KIN 的 frames 计数判断通断。
#
# 接线（本机实测：用 OpenMV 扩展板，板上标 "RX" 的那根内部接到 P4）：
#     扩展板 "RX"（= OpenMV P4 = UART3_TX）──▶ STM32 PB11 (USART3_RX)
#     GND ──────────────────────────────────── STM32 GND   ← 必须共地
#   本脚本只发不收，所以 PB10 那根回程线不需要接。
#   115200 8N1，与固件 USART3 一致。
#
# ⚠ 丝印不可靠：扩展板 / USB-TTL 模块上的 TX/RX 常按"对端视角"标注，
#   会出现"标 RX 的脚反而是本板输出"的情况。判断 TX 的唯一标准是
#   「接上 USB-TTL 的 RXD 后能收到数据的那根脚」，接 STM32 就用它。
#
# 部署：把本文件存到 OpenMV 的 U 盘并命名为 main.py，上电自动运行。
#
# ⚠ set_hmirror(True) 会把图像水平镜像。若上机后发现机械臂水平方向
#   越追越远（跟反了），把它改成 False —— 镜像会翻转 cx 的符号，
#   而 STM32 侧是拿 cx 直接当"左负右正"用的。
# ============================================================================
import sensor, image, time, ustruct
from pyb import UART, LED

# ---------------------------------------------------------------- 可调参数 --
DEPTH_MM       = 200     # 目标到相机距离，mm（0..255）。接测距后改成实测值
SEND_DEPTH     = True    # True=发 7 字节带深度；False=发 6 字节用固件默认值
USE_FLASH      = False   # 补光/闪光灯。颜色识别必须关：开灯会让 LAB 阈值整体偏移
USE_LENS_CORR  = True    # 畸变校正。关掉更快，但像素坐标与标定路径不再一致
DEBUG_PRINT    = False   # 调试打印。USB 打印会严重拖慢帧率，正常跑请保持 False
SEND_EMPTY     = False   # 已由 ALWAYS_SEND 取代，保留仅为兼容，勿再设 True
ALWAYS_SEND    = True    # 发送帧时若没有目标，就发 'A' 心跳（False = 该帧干脆不发）
SEND_EVERY     = 5       # 每隔多少帧发一次（1 = 每帧都发；5 = 5 帧发 1 帧）

# 阈值：某一次特定光照下标定的值。换灯、换环境、开补光都必须重新取。
blue_threshold   = (0, 59, 2, 81, -44, -35)
yellow_threshold = (0, 88, 10, -65, 20, 127)
red_threshold    = (11, 100, 8, 102, 15, 127)
green_threshold  = (4, 44, -69, -13, -10, 39)

# ------------------------------------------------------------------ 初始化 --
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)      # 160x120，对应固件 KIN_CAM_W / KIN_CAM_H
sensor.set_hmirror(True)                # 见文件头：方向跟反了就改 False
sensor.set_vflip(True)
# 2000 帧要等几十秒，200 帧足够让自动曝光稳定，上电更快能看到画面
sensor.skip_frames(n=200)
sensor.set_auto_gain(False)             # 颜色识别必须关自动增益
sensor.set_auto_whitebal(False)         # 以及自动白平衡

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# 关掉补光/闪光灯（有的型号没有这个 LED，忽略异常即可）
try:
    _flash = LED(4)
    if USE_FLASH:
        _flash.on()
    else:
        _flash.off()
except Exception:
    _flash = None


# ------------------------------------------------------------------ 工具函数 --
def send_detection(color, cx, cy):
    """把一次检测打包成视觉帧发给 STM32。"""
    d = DEPTH_MM
    if d < 0:
        d = 0
    elif d > 255:
        d = 255
    if SEND_DEPTH:
        # <BBBBBBB = 7 个无符号字节
        uart.write(ustruct.pack("<BBBBBBB", 0x2C, 0x12, color, cx, cy, d, 0x5B))
    else:
        uart.write(ustruct.pack("<BBBBBB", 0x2C, 0x12, color, cx, cy, 0x5B))
    if DEBUG_PRINT:
        print("frame color=%c cx=%d cy=%d depth=%d" % (color, cx, cy, d))


def largest(blobs):
    """同一颜色可能检出多个块，只取面积最大的那个，避免一帧发多帧。"""
    if not blobs:
        return None
    best = blobs[0]
    for b in blobs:
        if b[4] > best[4]:          # b[4] = 像素数
            best = b
    return best


# 优先级：蓝 > 黄 > 绿 > 红（沿用原脚本的顺序）
TARGETS = (
    (ord('B'), blue_threshold,   (0, 0, 255)),
    (ord('Y'), yellow_threshold, (255, 255, 0)),
    (ord('G'), green_threshold,  (0, 255, 0)),
    (ord('R'), red_threshold,    (255, 0, 0)),
)


# ------------------------------------------------------------------- 主循环 --
frame_count = 0
send_every = SEND_EVERY if SEND_EVERY > 0 else 1

while True:
    img = sensor.snapshot()
    if USE_LENS_CORR:
        # 老固件里 lens_corr 可能原地修改并返回 None，这里两种行为都兜住
        try:
            corrected = img.lens_corr(strength=1.8, zoom=1.0)
            if corrected is not None:
                img = corrected
        except Exception:
            pass

    frame_count += 1

    # 每帧都做识别（保证预览和最新坐标一直是最新的），
    # 但只在每 send_every 帧写一次串口，避免刷屏和占用带宽。
    found = None
    for color, threshold, draw_color in TARGETS:
        blobs = img.find_blobs([threshold],
                               x_stride=30, y_stride=25,
                               pixels_threshold=100)
        b = largest(blobs)
        if b is None:
            continue

        # 只在图像上标出最终上报的那一个，便于肉眼确认
        img.draw_rectangle((b[0], b[1], b[2], b[3]), color=draw_color)
        img.draw_cross(b[5], b[6], size=2, color=draw_color)
        found = (color, b[5], b[6])
        break               # 一种颜色命中即可，其余颜色让位给下一帧

    if (frame_count % send_every) != 0:
        continue            # 未到发送帧：只更新画面，不写串口

    if found is not None:
        send_detection(found[0], found[1], found[2])
    elif ALWAYS_SEND:
        # 该发送帧但没有目标：发 'A' 心跳（cx=cy=0），固件只计帧数、不解算
        send_detection(ord('A'), 0, 0)
