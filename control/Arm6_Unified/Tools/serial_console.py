"""串口调试台。不会自动 ON，也不会自动进入视觉闭环。

安装：python -m pip install pyserial
运行：python Tools/serial_console.py --port COM5

命令：
    ping | status | on | off | stop | home
    s <1-6> <角度>            单路调试
    pose <a1> <a2> <a3> <a4> <a5> <a6>
    auto 0|1                  退出/进入视觉闭环（需先 on）
    target <u> <v> [depth]    手动注入一次检测并解算
    kin                       查看闭环计数、最近目标与解出的关节角
    quit
"""
import argparse
import threading
from decimal import Decimal, InvalidOperation


def angle(text):
    try:
        value = Decimal(text)
    except InvalidOperation:
        raise ValueError("角度必须是数字") from None
    if not value.is_finite() or not 0 <= value <= 180:
        raise ValueError("角度必须在 0..180")
    if value != value.quantize(Decimal("0.1")):
        raise ValueError("最多一位小数")
    return format(value, ".1f")


def pixel(text, limit):
    value = int(text)
    if not 0 <= value <= limit:
        raise ValueError("像素必须在 0..%d" % limit)
    return str(value)


def command(line):
    parts = line.split()
    if not parts:
        return None
    name = parts[0].lower()

    if name in ("on", "off", "stop", "home", "status", "ping", "kin") and len(parts) == 1:
        return "$" + name.upper() + "#"

    if name == "auto" and len(parts) == 2:
        if parts[1] not in ("0", "1"):
            raise ValueError("auto 参数只能是 0 或 1")
        return "$AUTO,%s#" % parts[1]

    if name == "target" and len(parts) in (3, 4):
        frame = "$TARGET,%s,%s" % (pixel(parts[1], 255), pixel(parts[2], 255))
        if len(parts) == 4:
            depth = int(parts[3])
            if not 0 <= depth <= 65535:
                raise ValueError("depth 必须在 0..65535")
            frame += ",%d" % depth
        return frame + "#"

    if name == "s" and len(parts) == 3:
        ch = int(parts[1])
        if not 1 <= ch <= 6:
            raise ValueError("通道必须在 1..6")
        return "$S,%d,%s#" % (ch, angle(parts[2]))

    if name == "pose" and len(parts) == 7:
        return "$" + ",".join(angle(v) for v in parts[1:]) + "#"

    raise ValueError("可用: ping/status/on/off/stop/home/kin; s 1 95; "
                     "pose 90 90 90 90 90 90; auto 0|1; target 80 60 [200]; quit")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    import serial  # 延迟导入，便于 --help 不装 pyserial 也能看

    stop = threading.Event()
    lock = threading.Lock()

    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1) as port:
        def send(frame):
            with lock:
                port.write(frame.encode("ascii"))

        def receive():
            pending = bytearray()
            while not stop.is_set():
                try:
                    data = port.read(128)
                    if not data:
                        continue
                    pending.extend(data)
                    while b"\n" in pending:
                        line, _, rest = pending.partition(b"\n")
                        pending[:] = rest
                        text = line.decode("ascii", errors="replace").strip()
                        if text and text != "$PONG#":
                            print("\nSTM32:", text, flush=True)
                    if len(pending) > 2048:
                        pending.clear()
                except serial.SerialException as exc:
                    print("串口断开:", exc, flush=True)
                    stop.set()

        def heartbeat():
            # 2 s 是固件的链路超时，500 ms 心跳留 4 倍余量
            while not stop.wait(0.5):
                try:
                    send("$PING#")
                except serial.SerialException:
                    stop.set()

        workers = [threading.Thread(target=receive, daemon=True),
                   threading.Thread(target=heartbeat, daemon=True)]
        for worker in workers:
            worker.start()

        print("已连接。PWM 在你输入 on 之前保持 OFF；视觉闭环需再输入 auto 1。")
        print("命令: on, s 1 95, pose 90 90 90 90 90 90, auto 1, target 80 60 200, kin, stop, home, status, off, quit")

        try:
            while not stop.is_set():
                text = input("arm> ").strip()
                if text.lower() in ("quit", "exit"):
                    break
                try:
                    frame = command(text)
                    if frame:
                        send(frame)
                except (ValueError, serial.SerialException) as exc:
                    print("错误:", exc)
        except (EOFError, KeyboardInterrupt):
            pass
        finally:
            # 保持当前设定位置；不要在有重力负载时直接掉 PWM
            try:
                send("$STOP#")
            except serial.SerialException:
                pass
            stop.set()
            for worker in workers:
                worker.join(timeout=1)
            print("\n已关闭。STOP 保留 PWM；OFF 会撤掉控制脉冲。")


if __name__ == "__main__":
    main()
