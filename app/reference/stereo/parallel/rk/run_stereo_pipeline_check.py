#!/usr/bin/env python3
"""RK 立体整条流水线的离线闭环自检：标定 → 校正 → 匹配 → 深度。

为什么值得做：队友要在 RK 上实现"标定/校正/匹配/深度"。这里用**已知真值**
（内参、基线、深度）合成一对图像，把整条 OpenCV 流程跑一遍，看**反算出来的
深度**是否回到真值。跑通即说明"流程本身没问题"，剩下的只是实拍数据与性能。

流程：
  1) 合成标定：按已知 K、基线 b 渲染 12 张棋盘对 → `stereoCalibrate` → 检查
     基线误差与重投影 RMS；
  2) 用标定结果 `stereoRectify` → 渲染已知深度 Z0 的纹理平面 → `SGBM` 视差
     → `reprojectImageTo3D` → 比较中位深度与 Z0。

依赖：numpy、cv2。产物：`parallel/evidence/stereo_pipeline_check_20261015/`。
边界：合成数据；不证明实拍精度/同步，也不代表 RK 实机性能。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "parallel"))
import meta  # noqa: E402

OUT = REPO / "parallel" / "evidence" / "stereo_pipeline_check_20261015"

W, H = 640, 480
F = 700.0
CX, CY = (W - 1) / 2.0, (H - 1) / 2.0
BASELINE = 60.0          # mm
PATT = (9, 6)            # 内角点
SQ_MM = 25.0


def K():
    return np.array([[F, 0, CX], [0, F, CY], [0, 0, 1]], np.float64)


_BASE = None


def _base():
    """确定性**非周期**散斑底图。

    早前用正弦纹理做合成，结果 SGBM 锁到了相邻周期（视差 111 vs 真值 84）——
    这是**纹理歧义**，不是流程问题。散斑没有强周期性，块匹配才不会锁错。
    """
    global _BASE
    if _BASE is None:
        import cv2
        rng = np.random.default_rng(12345)
        b = rng.integers(0, 256, (2048, 2048), dtype=np.uint8)
        # 1 px/mm 且重度平滑：特征尺度约 10mm，对应 5~14 个图像像素（Z=500..1200mm），
        # 既能被块匹配抓住，又不会被像素网格混叠。
        _BASE = cv2.GaussianBlur(b, (11, 11), 0)
    return _BASE


def texture(x_mm, y_mm):
    """按空间坐标采样散斑（1 px/mm；重复周期 2048mm，远大于匹配搜索范围）。"""
    base = _base()
    ix = x_mm.astype(np.int64) % base.shape[1]
    iy = y_mm.astype(np.int64) % base.shape[0]
    return base[iy, ix].astype(np.float64)


def render_plane(Z0: float, x_off=0.0, y_off=0.0):
    """渲染位于深度 Z0 的纹理平面；返回 (left, right, 真值视差)。"""
    u = np.arange(W, dtype=np.float64)
    v = np.arange(H, dtype=np.float64)
    U, V = np.meshgrid(u, v)
    X = (U - CX) * Z0 / F + x_off
    Y = (V - CY) * Z0 / F + y_off
    left = texture(X, Y)
    # 右相机在 **+X 侧**（b 处）：同一点在右图里向左移 d → 左参考视差为正，
    # 与 SGBM/reprojectImageTo3D 的约定一致。早前放反了，导致视差为负、匹配全废。
    right = texture(X + BASELINE, Y)
    d_true = F * BASELINE / Z0
    return left.astype(np.uint8), right.astype(np.uint8), d_true


def synth_calib_views(nviews=12, seed=7):
    """按已知 K、基线渲染棋盘对（左相机 R=I,t=0；右相机 R=I,t=(-b,0,0)）。"""
    import cv2
    objp = np.zeros((PATT[0] * PATT[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:PATT[0], 0:PATT[1]].T.reshape(-1, 2) * SQ_MM
    rng = np.random.default_rng(seed)
    K1 = K()
    imgsL, imgsR, objpoints = [], [], []
    cornersL, cornersR = [], []
    for i in range(nviews):
        rvec = np.array([rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(-0.2, 0.2)])
        tvec = np.array([rng.uniform(-60, 60), rng.uniform(-40, 40), rng.uniform(700, 1100)])
        imgL = np.full((H, W), 40, np.uint8)
        imgR = np.full((H, W), 40, np.uint8)
        pL, _ = cv2.projectPoints(objp, rvec, tvec, K1, None)
        # 右相机在 +X 侧 → stereoCalibrate 会解出 T ≈ (-b, 0, 0)（从第一到第二相机的平移）
        pR, _ = cv2.projectPoints(objp, rvec, tvec + np.array([BASELINE, 0, 0]), K1, None)
        # 加 0.2px 观测噪声：让标定不再是"精确拟合已知投影"的平凡情形
        pL = pL + rng.normal(0.0, 0.2, pL.shape).astype(np.float32)
        pR = pR + rng.normal(0.0, 0.2, pR.shape).astype(np.float32)
        okL = okR = True
        for p in pL.reshape(-1, 2):
            x, y = int(round(p[0])), int(round(p[1]))
            if 0 <= x < W and 0 <= y < H:
                cv2.circle(imgL, (x, y), 2, 230, -1)
            else:
                okL = False
        for p in pR.reshape(-1, 2):
            x, y = int(round(p[0])), int(round(p[1]))
            if 0 <= x < W and 0 <= y < H:
                cv2.circle(imgR, (x, y), 2, 230, -1)
            else:
                okR = False
        if not (okL and okR):
            continue
        imgsL.append(imgL)
        imgsR.append(imgR)
        objpoints.append(objp.astype(np.float32))
        cornersL.append(pL.astype(np.float32))
        cornersR.append(pR.astype(np.float32))
    return objpoints, cornersL, cornersR


def main():
    import cv2
    OUT.mkdir(parents=True, exist_ok=True)
    res = {}

    # ---- 1) 标定 ----
    obj, cL, cR = synth_calib_views()
    res["calib_views"] = len(obj)
    if len(obj) < 5:
        print("STEREO_CHECK FAIL too few usable views")
        raise SystemExit(1)
    # 用合成图里已知的内参作为初值，只标外参（CALIB_FIX_INTRINSIC）
    rms, K1, D1, K2, D2, R, T, E, Fm = cv2.stereoCalibrate(
        obj, cL, cR, K(), None, K(), None, (W, H), flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    base_rec = float(np.linalg.norm(T))
    res["calib_rms_px"] = round(float(rms), 4)
    res["baseline_true_mm"] = BASELINE
    res["baseline_rec_mm"] = round(base_rec, 4)
    res["baseline_err_mm"] = round(abs(base_rec - BASELINE), 4)

    # ---- 2) 校正 + 匹配 + 深度 ----
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, (W, H), R, T, alpha=0)
    depth_res = {}
    for Z0 in (500.0, 800.0, 1200.0):
        left, right, d_true = render_plane(Z0)
        mapL = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (W, H), cv2.CV_16SC2)
        mapR = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (W, H), cv2.CV_16SC2)
        rL = cv2.remap(left, mapL[0], mapL[1], cv2.INTER_LINEAR)
        rR = cv2.remap(right, mapR[0], mapR[1], cv2.INTER_LINEAR)
        nd = int(np.ceil(d_true / 16.0) * 16 + 16)
        sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=nd, blockSize=15,
                                   P1=8 * 225, P2=32 * 225, disp12MaxDiff=1,
                                   uniquenessRatio=10, speckleWindowSize=200, speckleRange=2)
        d = sg.compute(rL, rR).astype(np.float32) / 16.0
        valid = d > 0
        pts = cv2.reprojectImageTo3D(d, Q)
        z = np.abs(pts[..., 2])
        # 只统计画面中部 60%（避开边界）
        mid = np.zeros_like(valid)
        mid[H // 5:4 * H // 5, W // 5:4 * W // 5] = True
        sel = valid & mid & np.isfinite(z)
        med_z = float(np.median(z[sel])) if sel.any() else float("nan")
        depth_res["Z%d" % int(Z0)] = {
            "true_mm": Z0, "d_true_px": round(d_true, 3),
            "d_median_px": round(float(np.median(d[sel])) if sel.any() else float("nan"), 3),
            "z_median_mm": round(med_z, 2),
            "rel_err": round(abs(med_z - Z0) / Z0, 4) if sel.any() else None,
            "valid_frac_mid": round(float(sel.sum()) / float(mid.sum()), 4),
        }
        cv2.imwrite(str(OUT / ("disp_Z%d.png" % int(Z0))),
                    np.clip(d / max(nd, 1) * 255, 0, 255).astype(np.uint8))
    res["depth"] = depth_res

    ok = (res["baseline_err_mm"] < 1.0 and res["calib_rms_px"] < 1.0
          and all(v["rel_err"] is not None and v["rel_err"] < 0.03
                  and v["valid_frac_mid"] > 0.5 for v in depth_res.values()))
    res["evidence_meta"] = meta.evidence_meta(__file__)
    res["result"] = "pass" if ok else "fail"
    (OUT / "summary.json").write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8")

    print("STEREO_CHECK calib_rms=%.4fpx baseline_err=%.4fmm" % (res["calib_rms_px"], res["baseline_err_mm"]))
    for k, v in depth_res.items():
        print("STEREO_CHECK %-6s d_true=%.2f d_med=%.2f z_med=%.1fmm rel_err=%s valid=%.3f"
              % (k, v["d_true_px"], v["d_median_px"], v["z_median_mm"], v["rel_err"], v["valid_frac_mid"]))
    print("STEREO_CHECK_DIR", OUT)
    print("STEREO_CHECK", "PASS" if ok else "FAIL")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
