"""主机侧回归测试：编译并运行 Tests/ 下的三个用例。

不参与 Keil 固件编译，只在 PC 上跑，用来验证纯逻辑部分
（视觉帧解析、关节映射、舵机协议、插补、正逆解精度）。

用法：
    python Tools/test_host.py                 # 自动找 gcc/clang
    python Tools/test_host.py --cc C:/mingw64/bin/gcc.exe
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KIN = ["kin/mat4.c", "kin/arm_fk.c", "kin/arm_ik.c", "kin/handeye.c"]
CORE_LOGIC = ["Core/Src/arm_config.c", "Core/Src/protocol.c",
              "Core/Src/pipeline.c", "Core/Src/vision_frame.c"]
INCLUDES = ["-ICore/Inc", "-Ikin"]

CASES = [
    ("test_kin", ["Tests/test_kin.c"] + KIN),
    ("test_arm", ["Tests/test_arm.c", "Core/Src/arm.c"] + CORE_LOGIC + KIN),
    ("test_pipeline", ["Tests/test_pipeline.c", "Core/Src/arm.c"] + CORE_LOGIC + KIN),
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default=shutil.which("gcc") or shutil.which("clang"))
    args = parser.parse_args()

    if not args.cc:
        print("ERROR: 找不到 C 编译器，请用 --cc 指定 gcc/clang 的完整路径")
        return 2

    build = ROOT / "build"
    build.mkdir(exist_ok=True)
    suffix = ".exe" if sys.platform == "win32" else ""
    failed = 0

    for name, sources in CASES:
        exe = build / (name + suffix)
        cmd = [args.cc, "-std=c11", "-O1", "-Wall", "-Wextra", "-pedantic"]
        cmd += INCLUDES + sources + ["-o", str(exe)]
        if sys.platform != "win32":
            cmd.append("-lm")
        print("\n=== build %s ===" % name)
        result = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        if result.stdout.strip():
            print(result.stdout.strip())
        if result.stderr.strip():
            print(result.stderr.strip())
        if result.returncode != 0:
            print("BUILD FAILED: %s" % name)
            failed += 1
            continue

        print("=== run %s ===" % name)
        run = subprocess.run([str(exe)], cwd=ROOT, capture_output=True, text=True)
        print((run.stdout + run.stderr).rstrip())
        if run.returncode != 0:
            print("RUN FAILED: %s (exit %d)" % (name, run.returncode))
            failed += 1

    print("\n%s" % ("ALL HOST TESTS PASSED" if failed == 0 else "%d CASE(S) FAILED" % failed))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
