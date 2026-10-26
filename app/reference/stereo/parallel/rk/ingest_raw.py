#!/usr/bin/env python3
"""真实 RAW dump 接入：任意常见 dump 格式 → 帧合同 .bin + 预览 + 统计。

用途（**进度推进**）：主线一旦从 FPGA 导出真实 RAW10 数据，本工具用一条命令把它
变成三样东西，供 RK 侧/压缩/标定直接使用：
  1. 符合 `parallel/frame` 合同的 `frame_cam*.bin`（带 CRC、几何、相位）；
  2. 可直接看的灰度预览（PGM + PNG）；
  3. 与项目一致的统计（A/B/C/D 四个 CFA 位置均值、min/max/σ、直方图摘要）。

支持输入格式：
  packed      MIPI RAW10，4px/5B（与帧合同一致）
  u16le       每样本 16bit 小端，10bit 在低位 [9:0]
  u16be       同上但大端
  raw12packed RAW12 3B→2样本（封存分支备用）

例：
  python parallel/rk/ingest_raw.py --raw dump.bin --width 1920 --height 1080 \
      --format packed --phase RGGB --cam A --out parallel/evidence/real_frame_2026xxxx

边界：软件工具；不证明采集质量，也不替代真实几何核对（WC/行数/CRC 仍要按接收报告读）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "frame"))
sys.path.insert(0, str(_HERE.parent))
import frame_contract as FC   # noqa: E402
import frame_io as FIO        # noqa: E402
import meta                   # noqa: E402


def load_samples(path: Path, fmt: str, width: int, height: int) -> np.ndarray:
    n = width * height
    data = path.read_bytes()
    if fmt == "packed":
        need = width // 4 * 5 * height
        if len(data) < need:
            raise ValueError("packed dump too short: %d < %d" % (len(data), need))
        return FIO.raw10_to_u16(data[:need], width, height)
    if fmt in ("u16le", "u16be"):
        if len(data) < 2 * n:
            raise ValueError("u16 dump too short: %d < %d" % (len(data), 2 * n))
        dt = "<u2" if fmt == "u16le" else ">u2"
        arr = np.frombuffer(data[:2 * n], dtype=dt).astype(np.uint16).reshape(height, width)
        if arr.max(initial=0) > 1023:
            raise ValueError("u16 dump has values > 1023 (not 10bit)")
        return arr
    if fmt == "raw12packed":
        need = width // 2 * 3 * height
        if len(data) < need:
            raise ValueError("raw12 dump too short")
        b = np.frombuffer(data[:need], dtype=np.uint8).reshape(-1, 3).astype(np.uint16)
        hi = (b[:, 0] << 4) | (b[:, 2] & 0x0F)
        lo = (b[:, 1] << 4) | ((b[:, 2] >> 4) & 0x0F)
        inter = np.empty(2 * len(b), dtype=np.uint16)
        inter[0::2], inter[1::2] = hi, lo
        return inter.reshape(height, width) >> 2          # 12→10 仅用于预览
    raise ValueError("unknown format %r" % fmt)


def cfa_stats(raw: np.ndarray, phase: int) -> dict:
    h, w = raw.shape
    yy, xx = np.mgrid[0:h, 0:w]
    tl, tr, bl, br = {"RGGB": ("R", "G", "G", "B"), "BGGR": ("B", "G", "G", "R"),
                      "GRBG": ("G", "R", "B", "G"), "GBRG": ("G", "B", "R", "G")}[
        FC.BAYER_PHASE_REV[phase]]
    mask = {(0, 0): tl, (0, 1): tr, (1, 0): bl, (1, 1): br}
    out = {}
    for (dy, dx), name in mask.items():
        sel = raw[(yy % 2 == dy) & (xx % 2 == dx)]
        out["pos_%s" % name] = {"mean": round(float(sel.mean()), 2),
                                "min": int(sel.min()), "max": int(sel.max()),
                                "count": int(sel.size)}
    return out


def write_pgm(path: Path, img: np.ndarray):
    h, w = img.shape
    with path.open("wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.astype(np.uint8).tobytes())


def ingest(raw_path: Path, out: Path, width: int, height: int, phase: str,
           fmt: str, cam: str, seq: int) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    raw = load_samples(raw_path, fmt, width, height)
    y8 = FIO.to_y8(raw, FC.BAYER_PHASE[phase], 10)

    # 1) 帧合同 .bin
    blob = FC.build_frame(raw.reshape(-1).tolist(), width, height,
                          format_id=FC.FMT_RAW10_PACKED, phase=phase,
                          camera_id=0 if cam.upper() == "A" else 1,
                          session_id=1, capture_seq=seq, mode_id=1)
    frame_path = out / ("frame_cam%s.bin" % cam.upper())
    frame_path.write_bytes(blob)
    info = FC.parse_frame(blob)

    # 2) 预览
    write_pgm(out / "preview_y8.pgm", y8)
    try:
        from PIL import Image
        Image.fromarray(y8).save(out / "preview_y8.png")
        png = True
    except Exception:
        png = False

    stats = {
        "evidence_meta": meta.evidence_meta(__file__),
        "source": raw_path.name, "format": fmt, "width": width, "height": height,
        "phase": phase, "camera": cam,
        "raw_min": int(raw.min()), "raw_max": int(raw.max()),
        "raw_mean": round(float(raw.mean()), 2), "raw_std": round(float(raw.std()), 2),
        "saturation_frac_1023": round(float((raw == 1023).mean()), 6),
        "zero_frac": round(float((raw == 0).mean()), 6),
        "cfa": cfa_stats(raw, FC.BAYER_PHASE[phase]),
        "y8_hist_head": [int(x) for x in np.bincount(y8.reshape(-1), minlength=256)[:16]],
        "frame_bin": frame_path.name, "frame_bytes": len(blob),
        "frame_parse_ok": bool(info["crc_ok"] and info["payload_ok"] and info["geometry_ok"]),
        "preview_png": png,
    }
    (out / "stats.json").write_text(json.dumps(stats, indent=2, ensure_ascii=False), encoding="utf-8")
    return stats


def selftest(verbose=True):
    res = []
    vec = _HERE.parent / "evidence" / "frame_contract_20261015"
    src = vec / "frame_8x4_packed.bin"
    if not src.exists():
        return [("vectors_available", False)]
    blob = src.read_bytes()
    info = FC.parse_frame(blob)
    raw = np.array(info["pixels"], dtype=np.uint16).reshape(info["header"]["height"],
                                                            info["header"]["width"])
    tmp = _HERE.parent / "evidence" / "_ingest_selftest"
    tmp.mkdir(parents=True, exist_ok=True)
    # 用两种格式各导一份 dump，再接入，结果必须与原帧一致
    (tmp / "d_packed.bin").write_bytes(FC.pack_raw10(raw.reshape(-1).tolist()))
    (tmp / "d_u16le.bin").write_bytes(raw.astype("<u2").tobytes())
    for fmt in ("packed", "u16le"):
        st = ingest(tmp / ("d_%s.bin" % fmt), tmp / ("out_%s" % fmt),
                    info["header"]["width"], info["header"]["height"], "RGGB", fmt, "A", 1)
        res.append(("ingest_%s_ok" % fmt, st["frame_parse_ok"]))
        h = FC.unpack_header((tmp / ("out_%s" % fmt) / "frame_camA.bin").read_bytes())
        res.append(("ingest_%s_matches_raw" % fmt,
                    h["raw_crc32"] == info["header"]["raw_crc32"]))
    if verbose:
        for r in res:
            print("INGEST_SELFTEST", r)
    return res


def main():
    ap = argparse.ArgumentParser(description="真实 RAW dump 接入")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--raw")
    ap.add_argument("--width", type=int)
    ap.add_argument("--height", type=int)
    ap.add_argument("--phase", default="RGGB", choices=list(FC.BAYER_PHASE))
    ap.add_argument("--format", default="packed",
                    choices=["packed", "u16le", "u16be", "raw12packed"])
    ap.add_argument("--cam", default="A")
    ap.add_argument("--seq", type=int, default=1)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if not a.selftest and not a.raw:
        ap.error("need --raw or --selftest")
    if a.raw and not a.out:
        ap.error("--raw requires --out")

    if a.selftest:
        res = selftest()
        print("INGEST", "PASS" if all(x[1] for x in res) else "FAIL")
        if not all(x[1] for x in res):
            raise SystemExit(1)
        return
    st = ingest(Path(a.raw), Path(a.out), a.width, a.height, a.phase, a.format, a.cam, a.seq)
    print("INGEST_DIR", a.out)
    print("INGEST", "PASS" if st["frame_parse_ok"] else "FAIL",
          "raw_mean=%.1f max=%d sat=%.4f" % (st["raw_mean"], st["raw_max"], st["saturation_frac_1023"]))
    if not st["frame_parse_ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
