import sensor, image, time,math
import ustruct
from pyb import UART, LED
blue_threshold  = (3, 17, -20, 25, -35, -10)#设置蓝色阈值

yellow_threshold  = (69, 89, -27, -4, -2, 127)#设置黄色阈值

red_threshold = (11, 100, 8, 102, 15, 127)#设置红色阈值

green_threshold = (4, 44, -69, -13, -10, 39)#设置绿色阈值

sensor.reset() #初始化摄像头
sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565
sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
sensor.set_hmirror(True)#水平方向翻转
sensor.set_vflip(True)#垂直方向翻转
sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
sensor.set_auto_gain(False) #使用颜色识别时需要关闭自动自动增益
sensor.set_auto_whitebal(False)#使用颜色识别时需要关闭自动自动白平衡
clock = time.clock() #追踪帧率


uart = UART(3,115200)   #设置串口波特率，与stm32一致
uart.init(115200, bits=8, parity=None, stop=1 )


def sending_data(color,cx,cy):
    global uart;
    data = ustruct.pack("<BBBBBB",
    0x2c,0x12,int(color),int(cx),int(cy),0x5b)

    uart.write(data);
    for i in data:
        print("data的内容是：    ",hex(i))

while(True):

    #img = sensor.snapshot() # 从感光芯片获得一张图像
    img = sensor.snapshot().lens_corr(strength = 1.8, zoom = 1.0)#从感光芯片获得一张图像并且进行畸变校正
    blue_blobs = img.find_blobs([blue_threshold],x_stride=30, y_stride=25, pixels_threshold=100 )
    yellow_blobs = img.find_blobs([yellow_threshold], x_stride=30, y_stride=25, pixels_threshold=100 )
    red_blobs = img.find_blobs([red_threshold], x_stride=30, y_stride=25, pixels_threshold=100 )
    green_blobs = img.find_blobs([green_threshold], x_stride=30, y_stride=25, pixels_threshold=100)
   #image.find_blobs(thresholds, roi=Auto, x_stride=2, y_stride=1, invert=False, area_threshold=10, pixels_threshold=10, merge=False, margin=0, threshold_cb=None, merge_cb=None)


    if blue_blobs:
        color_status = ord('B')
        for r in blue_blobs:
    #画矩形：image.draw_rectangle(rect_tuple, color=White)的格式是 (x, y, w, h)
    #画圆image.draw_circle(x, y, radius, color=White) 在图像中画一个圆。x,y是圆心坐标，radius是圆的半径
    #画十字：image.draw_cross(x, y, size=5, color=White) 在图像中画一个十字，x,y是坐标，size是两侧的尺寸
    #写字：image.draw_string(x, y, text, color=White) 在图像中写字8x10的像素,x,y是坐标。使用\n, \r, and \r\n会使光标移动到下一行。text是要写的字符串。
            img.draw_rectangle((r[0],r[1],r[2],r[3]),color=(0,0,255))
            img.draw_cross(r[5], r[6],size=2,color=(0,255,0))
            img.draw_string(r[0], (r[1]-10), "blue", color=(0,0,255))
            print("中心X坐标",r[5],"中心Y坐标",r[6],"识别颜色类型","蓝色")
            sending_data(color_status,r[5],r[6])

    elif yellow_blobs:
        color_status = ord('Y')
        for y in yellow_blobs:
            img.draw_rectangle((y[0],y[1],y[2],y[3]),color=(255,255,255))
            img.draw_cross(y[5], y[6],size=2,color=(255,255,255))
            img.draw_string(y[0], (y[1]-10), "yellow", color=(255,255,0))
            print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","黄色")
            sending_data(color_status,y[5],y[6])
    elif green_blobs:
        color_status = ord('G')
        for g in green_blobs:
            img.draw_rectangle((g[0],g[1],g[2],g[3]),color=(0,255,0))
            img.draw_cross(g[5], g[6],size=2,color=(0,255,0))
            img.draw_string(g[0], (g[1]-10), "green", color=(0,255,0))
            print("中心X坐标",g[5],"中心Y坐标",g[6],"识别颜色类型","绿色")
            sending_data(color_status,g[5],g[6])
    elif red_blobs:
        color_status = ord('R')
        for r in red_blobs:
            img.draw_rectangle((r[0],r[1],r[2],r[3]),color=(255,0,0))
            img.draw_cross(r[5], r[6],size=2,color=(255,0,0))
            img.draw_string(r[0], (r[1]-10), "red", color=(255,0,0))
            print("中心X坐标",r[5],"中心Y坐标",r[6],"识别颜色类型","红色")
            sending_data(color_status,r[5],r[6])
    else:
        color_status = ord('A')
        pass


