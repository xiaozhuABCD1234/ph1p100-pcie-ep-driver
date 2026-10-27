#!/usr/bin/env python3
"""双目标定入口模板（真实标定板图片）。

**需要用户/队友提供实拍图片**，本文件不含任何相机参数，也不含本项目的实测结果。

用法：
  python parallel/rk/stereo_calib_template.py --left "L*.png" --right "R*.png" --cols 9 --rows 6 --square-mm 25
输出：
  <out>/stereo_calib.json   标定结果（K1/D1/K2/D2/R/T/E/F/重投影误差/基线）
  <out>/rectify.json        校正映射参数（R1/R2/P1/P2/Q）

要点（避免常见坑）：
  * 左右图必须**同名配对、同步采集**；棋盘完整可见、覆盖画面四角与中心。
  * `square-mm` 是实际格边长（毫米），决定后续深度的单位与尺度。
  * 分辨率/裁剪/镜头/安装变化后必须重新标定。
  * 标定与运行必须使用**同一**处理版本（灰度路径、方向、裁剪）。
  * 本脚本只做离线标定；实时性与 RK 上的耗时需另测。
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np


def find_corners(paths, pattern):
    import cv2
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    obj, img = [], []
    used = []
    for p in paths:
        img_i = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img_i is None:
            continue
        ok, corners = cv2.findChessboardCorners(img_i, pattern, None)
        if not ok:
            continue
        corners = cv2.cornerSubPix(
            img_i, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        obj.append(objp)
        img.append(corners)
        used.append(p)
    return obj, img, used


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", required=True, help="左图 glob，例如 'L*.png'")
    ap.add_argument("--right", required=True, help="右图 glob")
    ap.add_argument("--cols", type=int, default=9, help="内角点数（列）")
    ap.add_argument("--rows", type=int, default=6, help="内角点数（行）")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    import cv2
    L = sorted(glob.glob(a.left))
    R = sorted(glob.glob(a.right))
    if not L or not R:
        raise SystemExit("没有找到图片；请提供 --left/--right 的 glob。")
    if len(L) != len(R):
        raise SystemExit("左右图片数量不一致：%d vs %d" % (len(L), len(R)))

    pattern = (a.cols, a.rows)
    obj, imgL, usedL = find_corners(L, pattern)
    obj2, imgR, usedR = find_corners(R, pattern)
    n = min(len(obj), len(obj2))
    if n < 5:
        raise SystemExit("有效棋盘对不足（%d）；至少需要 5 对，建议 15 对以上。" % n)
    if usedL[:n] != usedR[:n]:
        print("WARN: 左右可用图片不是同一批，按顺序取前 %d 对" % n)

    h, w = cv2.imread(L[0], cv2.IMREAD_GRAYSCALE).shape[:2]
    size = (w, h)
    objp = obj[0]
    objp_mm = objp * a.square_mm          # 单位：毫米 → 深度单位为毫米
    rms, K1, D1, K2, D2, Rm, T, E, F = cv2.stereoCalibrate(
        [objp_mm] * n, imgL[:n], imgR[:n], None, None, None, None, size,
        flags=cv2.CALIB_FIX_INTRINSIC if False else 0,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5))
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, size, Rm, T, alpha=0)

    out = Path(a.out) if a.out else Path(__file__).resolve().parent / "_calib_out"
    out.mkdir(parents=True, exist_ok=True)
    res = {
        "pairs_used": n, "image_size": size, "pattern": pattern, "square_mm": a.square_mm,
        "rms_reprojection_px": float(rms),
        "baseline_mm": float(np.linalg.norm(T)),
        "K1": K1.tolist(), "D1": D1.tolist(), "K2": K2.tolist(), "D2": D2.tolist(),
        "R": Rm.tolist(), "T": T.tolist(),
    }
    (out / "stereo_calib.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
    (out / "rectify.json").write_text(json.dumps(
        {"R1": R1.tolist(), "R2": R2.tolist(), "P1": P1.tolist(), "P2": P2.tolist(),
         "Q": Q.tolist(), "roi1": roi1, "roi2": roi2}, indent=2), encoding="utf-8")
    print("CALIB_RMS_PX", round(float(rms), 4), "BASELINE_MM", round(float(np.linalg.norm(T)), 3))
    print("CALIB_OUT", out)
    print("提醒：分辨率/裁剪/镜头变化后必须重新标定；极线误差请另测。")


if __name__ == "__main__":
    main()
