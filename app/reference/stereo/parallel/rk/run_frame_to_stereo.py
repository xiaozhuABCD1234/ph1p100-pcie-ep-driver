#!/usr/bin/env python3
"""端到端离线闭环：帧合同 .bin → RK 解码 → 灰度 → 立体。

与 `run_rk_offline.py` 的区别：这里**不直接喂内存数组**，而是先按帧合同
（`parallel/frame/frame_contract.py`）把合成双目写成 .bin 文件，再用
`frame_io` 从文件读回。这样验证的是"**合同文件本身**能被 RK 侧正确消费"。

产物：`parallel/evidence/frame_to_stereo_20261015/`（含两个 .bin 与 summary.json）。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "frame"))
import frame_io as FIO          # noqa: E402
import frame_contract as FC     # noqa: E402
import run_rk_offline as RK     # noqa: E402

REPO = _HERE.parents[1]
OUT = REPO / "parallel" / "evidence" / "frame_to_stereo_20261015"


def gray_to_bayer_raw10(gray8: np.ndarray) -> np.ndarray:
    """8bit 灰度 → RAW10 的 Bayer 马赛克（R=G=B），模拟传感器对灰卡的输出。"""
    return (gray8.astype(np.uint16) << 2)


def main():
    import cv2
    OUT.mkdir(parents=True, exist_ok=True)
    left8, right8, gt = RK.synth_scene()
    H, W = left8.shape

    # 1) 按帧合同写出左右两路
    frames = {}
    for name, img in (("A", left8), ("B", right8)):
        px = gray_to_bayer_raw10(img).reshape(-1).tolist()
        blob = FC.build_frame(px, W, H, format_id=FC.FMT_RAW10_PACKED,
                              phase="RGGB", camera_id=0 if name == "A" else 1,
                              session_id=7, capture_seq=11, mode_id=1)
        p = OUT / ("frame_cam%s.bin" % name)
        p.write_bytes(blob)
        frames[name] = p

    # 2) 从文件读回并解出灰度
    grays = {}
    for name, p in frames.items():
        info = FIO.load_frame_bin(p)
        h = info["header"]
        assert info["crc_ok"], "frame %s CRC fail" % name
        assert (h["width"], h["height"]) == (W, H)
        grays[name] = FIO.to_y8(info["raw"], phase=h["bayer_phase"], bits=10)

    # 3) 与"直接内存路径"必须一致（证明文件往返无损）
    direct = {"A": RK.to_gray_raw10(left8), "B": RK.to_gray_raw10(right8)}
    roundtrip_ok = all(np.array_equal(grays[k], direct[k]) for k in grays)

    # 4) 立体
    sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=RK.NUM_DISP,
                               blockSize=RK.BLOCK, P1=8 * RK.BLOCK ** 2,
                               P2=32 * RK.BLOCK ** 2, disp12MaxDiff=1,
                               uniquenessRatio=10, speckleWindowSize=100, speckleRange=2)
    d = sg.compute(grays["A"], grays["B"]).astype(np.float32) / 16.0
    metrics = RK.eval_disp(np.where(d > 0, d, -1), gt)
    cv2.imwrite(str(OUT / "disp_from_files.png"),
                np.clip(d / RK.NUM_DISP * 255, 0, 255).astype(np.uint8))

    ok = roundtrip_ok and metrics.get("bad2_ratio_in_valid", 1) < 0.35
    summary = {
        "frames": {k: v.name for k, v in frames.items()},
        "file_roundtrip_matches_direct_path": bool(roundtrip_ok),
        "sgbm_from_files": metrics,
        "result": "pass" if ok else "fail",
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print("FRAME_TO_STEREO_DIR", OUT)
    print("FRAME_TO_STEREO roundtrip=%s sgbm=%s" % (roundtrip_ok, json.dumps(metrics, ensure_ascii=False)))
    print("FRAME_TO_STEREO", "PASS" if ok else "FAIL")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
