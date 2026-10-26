#!/usr/bin/env python3
"""RK 侧离线立体闭环：合成双目 → 灰度 → StereoBM/SGBM → 视差误差。

用途：给队友（RK 执行者）一份**立刻能跑**的离线基线，并且它用的灰度路径与
FPGA 金参考逐像素一致（见 `parallel/rk/frame_io.py` 的自检）。队友只需要把
"合成双目"换成实拍左右图，其余流程不变。

不含：真实标定（需要实拍标定板）；标定入口见 `stereo_calib_template.py`。

依赖：numpy、cv2。产物写入 `parallel/evidence/rk_offline_<date>/`。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
import frame_io as FIO  # noqa: E402

REPO = _HERE.parents[1]
sys.path.insert(0, str(REPO / "parallel"))
import meta  # noqa: E402

OUT = REPO / "parallel" / "evidence" / "rk_offline_20261015"

W, H = 320, 240
D1, D2 = 16, 48          # 左半/右半的已知视差
NUM_DISP = 64
BLOCK = 15


def synth_scene():
    """确定性纹理场景 + 阶梯视差左右图 + 真值。"""
    rng = np.random.default_rng(20261015)
    scene = rng.integers(0, 256, size=(H, W + NUM_DISP), dtype=np.uint8)
    # 加一些结构，避免纯噪声
    scene[::16, :] = 255
    scene[:, ::16] = 0
    gt = np.zeros((H, W), np.int16)
    gt[:, : W // 2] = D1
    gt[:, W // 2:] = D2
    left = scene[:, 0:W].copy()
    right = np.empty((H, W), np.uint8)
    for x in range(W):
        right[:, x] = scene[:, x + int(gt[0, x])]
    return left, right, gt


def to_gray_raw10(img8: np.ndarray) -> np.ndarray:
    """把 8bit 灰度当作 RAW10 的 Bayer 马赛克（R=G=B），走一遍 FPGA 的灰度金路径。"""
    raw10 = (img8.astype(np.uint16) << 2)  # 0..1020，仍在 10bit
    return FIO.to_y8(raw10, phase=0, bits=10)


def eval_disp(disp, gt, valid_pad=3 * BLOCK):
    """在左右半区内部（避开中缝与边界）统计误差；disp 为 <0 表示无效。"""
    m = disp > 0
    x = np.arange(W)
    inner = (x >= valid_pad) & (x < W // 2 - 2 * BLOCK)
    inner |= (x >= W // 2 + 2 * BLOCK) & (x < W - valid_pad)
    m = m & inner[None, :]
    total = int(inner.sum()) * H
    valid = int(m.sum())
    if valid == 0:
        return {"total": total, "valid": 0, "valid_ratio": 0.0}
    err = np.abs(disp[m].astype(np.int32) - gt[m].astype(np.int32))
    return {
        "total": total, "valid": valid,
        "valid_ratio": round(valid / total, 4),
        "bad1_ratio_in_valid": round(float((err > 1).mean()), 4),
        "bad2_ratio_in_valid": round(float((err > 2).mean()), 4),
        "rmse": round(float(np.sqrt((err.astype(float) ** 2).mean())), 3),
    }


def main():
    import cv2
    OUT.mkdir(parents=True, exist_ok=True)
    left8, right8, gt = synth_scene()
    left_g, right_g = to_gray_raw10(left8), to_gray_raw10(right8)

    results = {}
    # StereoBM
    bm = cv2.StereoBM_create(numDisparities=NUM_DISP, blockSize=21)
    d_bm = bm.compute(left_g, right_g).astype(np.float32) / 16.0
    results["StereoBM"] = eval_disp(np.where(d_bm > 0, d_bm, -1), gt)
    # StereoSGBM
    sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=NUM_DISP, blockSize=BLOCK,
                               P1=8 * BLOCK * BLOCK, P2=32 * BLOCK * BLOCK,
                               disp12MaxDiff=1, uniquenessRatio=10,
                               speckleWindowSize=100, speckleRange=2)
    d_sg = sg.compute(left_g, right_g).astype(np.float32) / 16.0
    results["StereoSGBM"] = eval_disp(np.where(d_sg > 0, d_sg, -1), gt)

    # 真值与图像落盘（PNG）
    cv2.imwrite(str(OUT / "left_gray.png"), left_g)
    cv2.imwrite(str(OUT / "right_gray.png"), right_g)
    cv2.imwrite(str(OUT / "gt_disp.png"), (gt.astype(np.float32) / NUM_DISP * 255).astype(np.uint8))
    for name, d in (("bm", d_bm), ("sgbm", d_sg)):
        vis = np.clip(d / NUM_DISP * 255, 0, 255).astype(np.uint8)
        cv2.imwrite(str(OUT / ("disp_%s.png" % name)), vis)

    # 一致性前提：左半/右半真值差异应在结果里可见
    ok = all(r.get("valid_ratio", 0) > 0.5 and r.get("bad2_ratio_in_valid", 1) < 0.35
             for r in results.values())
    summary = {
        "evidence_meta": meta.evidence_meta(__file__),
        "geometry": {"width": W, "height": H, "numDisparities": NUM_DISP,
                     "gt_left_half": D1, "gt_right_half": D2},
        "note": "灰度经 frame_io 的金参考路径；结果仅供离线流程验证，不是 RK 实机性能",
        "results": results,
        "result": "pass" if ok else "fail",
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print("RK_OFFLINE_DIR", OUT)
    for k, v in results.items():
        print("RK_OFFLINE", k, json.dumps(v, ensure_ascii=False))
    print("RK_OFFLINE", "PASS" if ok else "FAIL")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
