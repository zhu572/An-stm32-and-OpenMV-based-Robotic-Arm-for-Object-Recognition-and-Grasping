"""把官方 STM32CubeF1 包里的 HAL / CMSIS / 启动文件 / system 复制进工程，
并把 .uvprojx 里的绝对路径改写为相对路径，使工程可以整体拷到别的机器。

背景：`control\\STM32_Arm6` 目录已被删除，工程现在直接引用
      C:\\Users\\zhu\\STM32Cube\\Repository\\STM32Cube_FW_F1_V1.8.7
绝对路径里带用户名，换机器/换用户名后跑一次本脚本即可。

用法：
    python Tools/materialize.py
    python Tools/materialize.py --source "D:/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.7"

做四件事：
  1) Drivers\\STM32F1xx_HAL_Driver  <- 官方包
  2) Drivers\\CMSIS                  <- 官方包
  3) Core\\Src\\system_stm32f1xx.c
  4) MDK-ARM\\startup_stm32f103xb.s
然后把两个 .uvprojx 里的绝对路径换成相对路径（并备份 .bak）。
"""
import argparse
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = Path(r"C:\Users\zhu\STM32Cube\Repository\STM32Cube_FW_F1_V1.8.7")

PROJECTS = [
    ROOT / "Arm6_Unified.uvprojx",
    ROOT.parent / "Arm6_Test_Servo1" / "Arm6_Test_Servo1.uvprojx",
]


def copy_into(src: Path, dst: Path) -> None:
    if not src.exists():
        raise SystemExit("缺少源文件/目录: %s" % src)
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.is_dir():
        shutil.copytree(src, dst, dirs_exist_ok=True)
    else:
        shutil.copy2(src, dst)
    print("copied  %s  ->  %s" % (src, dst))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=str(DEFAULT_SOURCE),
                        help="STM32Cube_FW_F1 包根目录")
    args = parser.parse_args()

    fw = Path(args.source)
    drv = fw / "Drivers"
    if not drv.is_dir():
        raise SystemExit("找不到官方包: %s" % fw)

    hal = drv / "STM32F1xx_HAL_Driver"
    cmsis = drv / "CMSIS"
    tmpl = cmsis / "Device" / "ST" / "STM32F1xx" / "Source" / "Templates"
    system_c = tmpl / "system_stm32f1xx.c"
    startup_s = tmpl / "arm" / "startup_stm32f103xb.s"

    copy_into(hal, ROOT / "Drivers" / "STM32F1xx_HAL_Driver")
    copy_into(cmsis, ROOT / "Drivers" / "CMSIS")
    copy_into(system_c, ROOT / "Core" / "Src" / "system_stm32f1xx.c")
    copy_into(startup_s, ROOT / "MDK-ARM" / "startup_stm32f103xb.s")

    replacements = [
        (str(hal) + "\\", "Drivers\\STM32F1xx_HAL_Driver\\"),
        (str(cmsis) + "\\", "Drivers\\CMSIS\\"),
        (str(system_c), "Core\\Src\\system_stm32f1xx.c"),
        (str(startup_s), "MDK-ARM\\startup_stm32f103xb.s"),
    ]

    for project in PROJECTS:
        if not project.is_file():
            print("跳过（不存在）: %s" % project)
            continue
        bak = project.with_suffix(".uvprojx.bak")
        shutil.copy2(project, bak)
        text = project.read_text(encoding="utf-8")
        patched = text
        for old, new in replacements:
            patched = patched.replace(old, new)
        if patched == text:
            print("已是相对路径，未改动: %s" % project.name)
        else:
            project.write_text(patched, encoding="utf-8")
            print("路径已改写: %s（备份 %s）" % (project.name, bak.name))

    print("\n完成。现在工程只依赖自己目录下的 Drivers/ 与 MDK-ARM/，可以整体拷走。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
