#!/usr/bin/env python3
"""跨语言一致性：C 参考解码器 vs Python/numpy 金参考（逐像素）。

这回答一个具体问题：**"换个语言/换个人实现，结果会不会一样？"**
若两条独立实现路径在全部相位、全部向量上逐字节一致，队友在 RK 侧用 C 复刻时
就有可依赖的参照；不一致就是语义没写清。

步骤：gcc 编译 `frame_decode.c` → 对每个共享向量跑一遍 → 与 `frame_io.to_y8` 比对。
负对照：故意改坏帧内容 → C 必须报错退出（非 0）。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
RK = REPO / "parallel" / "rk"
VEC = REPO / "parallel" / "evidence" / "frame_contract_20261015"
OUTROOT = REPO / "parallel" / "evidence" / "c_cross_check_20261015"
GCC = r"D:\MSYS2\mingw64\bin\gcc.exe"

sys.path.insert(0, str(RK))
sys.path.insert(0, str(REPO / "parallel" / "frame"))
import frame_io as FIO          # noqa: E402
import frame_contract as FC     # noqa: E402


def main():
    OUTROOT.mkdir(parents=True, exist_ok=True)
    exe = OUTROOT / "frame_decode.exe"
    if not Path(GCC).exists():
        print("C_CROSS_SKIP gcc not found at", GCC)
        raise SystemExit(2)

    comp = subprocess.run([GCC, "-O2", "-Wall", "-o", str(exe), str(RK / "frame_decode.c")],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)
    print(comp.stdout.decode("utf-8", errors="replace").encode("ascii", "replace").decode("ascii"))
    if comp.returncode != 0 or not exe.exists():
        print("C_CROSS_FAIL compile")
        raise SystemExit(1)

    results = []
    for name in ("frame_8x4_packed.bin", "frame_64x48_packed.bin"):
        src = VEC / name
        if not src.exists():
            continue
        blob = src.read_bytes()
        info = FC.parse_frame(blob)
        h = info["header"]
        raw = np.array(info["pixels"], dtype=np.uint16).reshape(h["height"], h["width"])
        y8_c = OUTROOT / (name + ".y8")
        r = subprocess.run([str(exe), str(src), str(y8_c)], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=120)
        text = r.stdout.decode("utf-8", errors="replace")
        c_bytes = y8_c.read_bytes() if y8_c.exists() else b""
        py_y8 = FIO.to_y8(raw, h["bayer_phase"], 10).tobytes()
        ok = (r.returncode == 0) and (c_bytes == py_y8)
        results.append((name, ok, text.strip()))
        print("C_CROSS", name, "PASS" if ok else "FAIL", "| C:", text.strip()[:60],
              "| py_sum:", int(np.frombuffer(py_y8, dtype=np.uint8).sum()))

    # 负对照：改坏一个载荷字节 → C 必须非 0 退出
    src = VEC / "frame_8x4_packed.bin"
    if src.exists():
        bad = bytearray(src.read_bytes())
        bad[128 + 3] ^= 0xFF
        badf = OUTROOT / "corrupt.bin"
        badf.write_bytes(bytes(bad))
        r = subprocess.run([str(exe), str(badf), str(OUTROOT / "corrupt.y8")],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        results.append(("corrupt_frame_rejected", r.returncode != 0,
                        r.stdout.decode("utf-8", errors="replace").strip()))

    ok = all(x[1] for x in results)
    print("C_CROSS_CHECK", "PASS" if ok else "FAIL", "cases=%d" % len(results))
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
