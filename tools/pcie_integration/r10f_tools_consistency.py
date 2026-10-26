#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T11.rk-align-raw：Rust 侧（rka1.rs::r10f）与 Python 侧（r10f_strict.py）字段偏移一致性交叉核对。

为什么要有它：两边的严格解析器是同一份合同的**两个离线实现**（RK 用 Rust、本工具链用
Python），本轮 Rust 无法在本机编译（缺 cargo/交叉工具链）。本脚本只做**静态**核对：
从两份源码里抽取「字段 → 头内偏移」映射并逐项比对，保证不会出现「Python 拒 0x2020、
Rust 读 0x2010」这类静默错位。

用法：python tools/pcie_integration/r10f_tools_consistency.py [--json-out FILE]
退出码：0 一致；1 有差异。
"""
import argparse
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
ROOT = HERE.parents[2]


def python_offsets(path):
    src = path.read_text(encoding="utf-8")
    out = {}
    for m in re.finditer(r'\(0x([0-9A-Fa-f]{2}),\s*"([a-z_0-9]+)",\s*"([^"]+)"\)', src):
        out[m.group(2)] = int(m.group(1), 16)
    return out


def rust_offsets(path):
    src = path.read_text(encoding="utf-8")
    out = {}
    for m in re.finditer(r'([a-z_0-9]+):\s*(?:le16|le32)\(\s*hdr,\s*0x([0-9A-Fa-f]+)\s*\)', src):
        out[m.group(1)] = int(m.group(2), 16)
    for m in re.finditer(r'([a-z_0-9]+):\s*hdr\[\s*0x([0-9A-Fa-f]+)\s*\]', src):
        out[m.group(1)] = int(m.group(2), 16)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--python-src", default=str(ROOT / "tools/pcie_integration/r10f_strict.py"))
    ap.add_argument("--rust-src", default=str(ROOT / "workflow/runs/20261017-011744-rk-fpga-align/"
                                             "raw/rk-driver-src/app/rs/src/rka1.rs"))
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    py = python_offsets(Path(args.python_src))
    rs = rust_offsets(Path(args.rust_src))
    alias = {"stride": "stride_bytes"}

    mismatches = []
    for name, off in sorted(rs.items()):
        pname = alias.get(name, name)
        if pname not in py:
            mismatches.append({"field": name, "issue": "not-in-python-table"})
        elif py[pname] != off:
            mismatches.append({"field": name, "issue": "offset 0x%02x != python 0x%02x" % (off, py[pname])})

    expected = set(py) - {"magic"}
    consumed = {alias.get(k, k) for k in rs}
    unconsumed = sorted(expected - consumed)

    out = {
        "rust_src": args.rust_src,
        "python_src": args.python_src,
        "rust_fields": len(rs),
        "python_fields": len(py),
        "mismatches": mismatches,
        "python_fields_not_read_by_rust": unconsumed,
        "marker": "R10F_TOOLS_CONSISTENT" if not mismatches else "R10F_TOOLS_MISMATCH",
    }
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(out, ensure_ascii=False))
    return 0 if not mismatches else 1


if __name__ == "__main__":
    sys.exit(main())
