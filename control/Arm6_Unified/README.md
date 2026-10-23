# Arm6_Unified —— 机械臂全流程单片工程（Keil MDK）

**目标**：把 `control/` 下散落的固件、PC 端运动学库、视觉脚本协议整合成**一个** Keil 工程，实现在 STM32F103C8T6 上跑完
`串口数据 → 手眼标定 → 正逆解 → 六路 PWM` 的全流程。

## 一、目录

```
Arm6_Unified/
├─ Arm6_Unified.uvprojx     ← 用 Keil 直接打开这个
├─ Core/
│  ├─ Inc/                  main.h app_hal.h arm.h arm_config.h protocol.h
│  │                        vision_frame.h pipeline.h stm32f1xx_hal_conf.h
│  └─ Src/                  main.c app_hal.c arm.c arm_config.c protocol.c
│                            vision_frame.c pipeline.c stm32f1xx_hal_msp.c stm32f1xx_it.c
├─ kin/                     运动学（float 移植）：mat4 arm_fk arm_ik handeye + kin_config.h
├─ Tests/                   主机侧测试（不参与 Keil 编译）
├─ Tools/                   test_host.py / serial_console.py / materialize.py
└─ Docs/                    架构与接线.md / 改动清单.md
```

**厂商代码来源**：HAL、CMSIS、启动文件、`system_stm32f1xx.c` 现在取自**官方 STM32CubeF1 V1.8.7 包**

```
C:\Users\zhu\STM32Cube\Repository\STM32Cube_FW_F1_V1.8.7\
```

（原先借用的 `../STM32_Arm6` 目录已被删除，故改为直接引用官方包。）
`stm32f1xx_hal_conf.h` 已复制进 `Core\Inc`，是官方模板的原样副本。

> ⚠️ 工程里含绝对路径（带用户名 `zhu`）。换机器 / 换用户名 / 重装 Cube 包后，
> 运行一次 `python Tools/materialize.py` 会把厂商代码复制进本工程并改成相对路径。

## 二、第一次编译

1. 双击 `Arm6_Unified.uvprojx`（Keil MDK，ARMCC V5.06，与旧工程同一套编译选项）。
2. 点 `Build`（F7）。
3. 期望 `0 Error(s), 0 Warning(s)`，产物在 `Objects\Arm6_Unified.hex`。

> 路径基准：本工程自己的文件是 `Core\Src\*.c`、`kin\*.c`（相对工程根目录），
> 厂商代码是 `C:\Users\zhu\STM32Cube\...` 绝对路径。两种混用，改路径时别搞混。
>
> 若报 `stm32f1xx_hal.h` / 某个 HAL `.c` 找不到 → Cube 包被移动了，
> 跑 `python Tools/materialize.py`。

## 三、接线

| 用途 | STM32 引脚 | 对端 |
|---|---|---|
| 电脑（ASCII 命令） | PA9 / PA10 (USART1) | USB-TTL，115200 8N1 |
| **OpenMV（视觉帧）** | **PB11 (USART3_RX)** | OpenMV 扩展板上标 "RX" 的那根脚（内部接 P4 = UART3_TX） |
| 舵机 S1–S4 | PA0–PA3 (TIM2 CH1–4) | 六路舵机，**独立电源** |
| 舵机 S5/S6 | PA6/PA7 (TIM3 CH1–2) | S6 一般是夹爪 |
| 调试 | PA13/PA14 (SWD) | ST-Link |

OpenMV 侧用的是它自己的 UART3（`view/main.py` 里的 `UART(3,115200)`），与 STM32 的 USART3 同名但毫无关系。

⚠️ **丝印不可靠**：扩展板 / USB-TTL 模块上的 `TX`/`RX` 常按"对端视角"标注。本机实测：扩展板上标 **`RX`** 的那根，内部接的是 OpenMV 的 **P4 = UART3_TX**，接 USB-TTL 的 `RXD` 才有数据。所以判断依据只有一条 —— **接上 USB-TTL 的 RXD 后能收到数据的那根就是 TX**，接 STM32 时把它接到 **PB11**。回程的 PB10 不需要（脚本只发不收）。

## 四、上电顺序

1. 只接一个空载舵机，先验证通信：`$PING#` → `$PONG#`，`$STATUS#` → `$STATE,...#`。
2. `$ON#` 使能 PWM，`$S,1,95#` 小幅动一下。
3. `$TARGET,80,60,200#` 手动跑一次完整解算（不依赖 OpenMV），看是否回 `$TGT,OK,...#`。
4. `$AUTO,1#` 进入视觉闭环，OpenMV 开始发帧后机械臂会跟着目标动。
5. `$STOP#` / `$OFF#` 立即退出闭环。

**`$ON#` 不会测量真实角度**，首次从机械实际位置到中位的运动不受插补控制。
无编码器时 `$STATUS#` 的角度是"软件设定值"而不是实测位置，别把 `$DONE#` 当成到位。

## 五、命令表（USART1，ASCII，`$` 开始 `#` 结束）

| 命令 | 说明 | 应答 |
|---|---|---|
| `$PING#` | 心跳，建议 500 ms | `$PONG#` |
| `$STATUS#` | 使能/运动/掉线 + 六路设定值(0.1°) | `$STATE,en,mv,lost,a1..a6#` |
| `$ON#` / `$OFF#` / `$STOP#` | 使能 / 关 PWM / 保持当前 | `$ACK,ON#` 等 |
| `$HOME#` | 平滑回配置初始角 | `$ACK,MOVE#` → `$DONE#` |
| `$S,1,95#` | 单路调试，其余保持 | 同上 |
| `$90,90,90,90,90,90#` | 六路直接给定 | 同上 |
| `$AUTO,1#` / `$AUTO,0#` | 进入 / 退出视觉闭环 | `$ACK,AUTO,1#` |
| `$TARGET,u,v[,depth]#` | 手动注入一次检测并解算 | `$TGT,OK,...#` / `$TGT,ERR,n#` |
| `$KIN#` | 闭环计数 + 最近目标 + 解出关节 | `$KIN,...#` |

`$TGT,ERR,n#` 的 n：`1` 反投影失败，`2` 逆解不可达/无限位解，`3` 接近方向残差超限。
`$KIN` 的字段顺序见 `Docs/架构与接线.md`。

## 六、视觉帧（USART3）

`view/main.py` 的帧格式：

```
0x2C 0x12 <color> <cx> <cy> [depth] 0x5B
```

- **7 字节（当前默认）**：带 depth（0–255 mm）。
- 6 字节：不带 depth，固件用 `ARM_DEFAULT_DEPTH_MM`（默认 200 mm，`[TEMP]`）。
- `color = 'A'` 是**无目标心跳帧**：固件只计入帧数、不解算，不会产生幻影动作。

发送节流由 `view/main.py` 的 `SEND_EVERY` 控制（默认 5，即每 5 帧发 1 帧；
识别仍每帧都做，画面预览不受影响）。带宽 ≈ 相机帧率 / `SEND_EVERY` × 7 字节。

## 七、必须替换的 `[TEMP]` 值

| 文件 | 内容 |
|---|---|
| `kin/kin_config.h` | 连杆 `KIN_P/A1..A4`、关节限位、相机内参 `fx/fy/cx/cy` |
| `kin/handeye.c` | `g_default_calib`（默认是 `translate(0,0,30)` 同轴安装） |
| `Core/Src/arm_config.c` | 每路脉宽/限位/`home`/`trim`/`reverse`、`g_joint_map` 零位与方向 |
| `Core/Inc/arm_config.h` | `ARM_DEFAULT_DEPTH_MM`、`ARM_KIN_MAX_DIR_ERR_MRAD`、`ARM_KIN_MIN_INTERVAL_MS` |

完整清单与标定步骤见 `Docs/架构与接线.md` 第 6 节。

## 八、已知限制

- 没有编码器：闭环用的是"软件设定角"，末端实际误差会累积。
- 4R 极坐标臂的末端 z 轴**只能在目标点方位角的竖直平面内**，相机光轴偏离该平面时必然有接近方向残差（默认同轴安装为 0）。
- depth 只有单字节 → 0–255 mm；工作空间更大就需要改协议（同时改 `view/main.py`）。
- 视觉帧无 CRC，一个合法字节被打扰成另一个合法字节无法检测。
- 固件未做跨定时器硬件相位同步，`$DONE#` 只是软件插补结束。

## 九、验收状态

- Keil 编译：**待你在本地 Build**，把日志回传（本环境 shell 不可用，无法代为编译）。
- 主机侧测试：`Tests/` 已写好，`python Tools/test_host.py` 可跑，同样**未在本环境执行**。
- 硬件：未上电、未接舵机、未联调 OpenMV。
