#!/usr/bin/env python3
"""RK 侧帧解码 / 去马赛克 / 灰度：与 FPGA 金参考逐像素一致。

用途：给 RK（或 PC 离线）一份**可直接运行**的解码路径，且与 `parallel/algorithms`
的纯 Python 金参考**逐像素相同**（本文件自检会断言这一点）。这样 FPGA 与 RK
两侧对同一份帧合同向量会得到同一组 Y8，避免"各算各的"。

依赖：numpy（必需）。本文件**不使用 cv2**，也**不用** cv2 的去马赛克替代——
cv2 的边界与插值语义与金参考不同，会破坏"逐像素一致"这一约束。

边界：软件参考，非 RTL，非实拍验证。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / "frame"))
sys.path.insert(0, str(_HERE.parent / "algorithms"))

import frame_contract as FC          # noqa: E402
import image_algorithms_ref as IA    # noqa: E402

_LUT4 = {0: ("R", "G", "G", "B"), 1: ("B", "G", "G", "R"),
         2: ("G", "R", "B", "G"), 3: ("G", "B", "R", "G")}


def raw10_to_u16(payload: bytes, width: int, height: int) -> np.ndarray:
    """packed RAW10（MIPI 4px/5B）→ (h,w) uint16 样本。"""
    if width <= 0 or height <= 0:
        raise ValueError("width/height must be positive")
    if width % 4:
        raise ValueError("packed RAW10 requires width multiple of 4")
    need = width // 4 * 5 * height
    if len(payload) < need:
        raise ValueError("payload too short for packed RAW10")
    b = np.frombuffer(payload[:need], dtype=np.uint8).reshape(-1, 5).astype(np.uint16)
    p0 = (b[:, 0] << 2) | (b[:, 4] & 0x3)
    p1 = (b[:, 1] << 2) | ((b[:, 4] >> 2) & 0x3)
    p2 = (b[:, 2] << 2) | ((b[:, 4] >> 4) & 0x3)
    p3 = (b[:, 3] << 2) | ((b[:, 4] >> 6) & 0x3)
    return np.stack([p0, p1, p2, p3], axis=1).reshape(height, width)


def _own_grid(width: int, height: int, phase: int):
    """每个像素自身的 CFA 颜色网格。"""
    tl, tr, bl, br = _LUT4[phase]
    yy, xx = np.mgrid[0:height, 0:width]
    top = (yy % 2) == 0
    lef = (xx % 2) == 0
    return np.where(top, np.where(lef, tl, tr), np.where(lef, bl, br))


def _left_intent_grid(width: int, height: int, phase: int):
    """(x-1, y) 处的**目标**颜色（不夹取、按奇偶）；x=0 时也正确。"""
    tl, tr, bl, br = _LUT4[phase]
    yy, xx = np.mgrid[0:height, 0:width]
    top = (yy % 2) == 0
    x_even = (xx % 2) == 0          # x 偶 -> x-1 为奇列
    return np.where(top, np.where(x_even, tr, tl), np.where(x_even, br, bl))


def _nb_index(dx: int, dy: int, w: int, h: int):
    """邻居索引，与金参考 `_clamp_color` 等价：夹回图像并保持 CFA 奇偶。"""
    yy, xx = np.mgrid[0:h, 0:w]
    tx, ty = xx + dx, yy + dy
    cx = np.clip(tx, 0, w - 1)
    needx = (cx % 2) != (tx % 2)
    canx = needx & ((cx + 1 <= w - 1) | (cx - 1 >= 0))
    cx = np.where(canx & (cx + 1 <= w - 1), cx + 1, np.where(canx, cx - 1, cx))
    cy = np.clip(ty, 0, h - 1)
    needy = (cy % 2) != (ty % 2)
    cany = needy & ((cy + 1 <= h - 1) | (cy - 1 >= 0))
    cy = np.where(cany & (cy + 1 <= h - 1), cy + 1, np.where(cany, cy - 1, cy))
    fail = (needx & ~canx) | (needy & ~cany)
    return np.where(fail, yy, cy), np.where(fail, xx, cx)


def demosaic_bilinear_np(raw: np.ndarray, phase: int) -> np.ndarray:
    """与金参考逐像素一致的 3x3 双线性 + **保色夹取**边界（向量化）。

    边界规则必须与 `image_algorithms_ref._clamp_color` 完全一致：夹回图像后
    保持 CFA 奇偶；只有图像某维为 1 像素时才退化为中心像素。
    """
    h, w = raw.shape
    src = raw.astype(np.int64)

    def S(dx, dy):
        cy, cx = _nb_index(dx, dy, w, h)
        return src[cy, cx]

    left, right, up, down = S(-1, 0), S(1, 0), S(0, -1), S(0, 1)
    ul, ur, dl, dr = S(-1, -1), S(1, -1), S(-1, 1), S(1, 1)

    avg2 = lambda a, b: (a + b + 1) // 2
    avg4 = lambda a, b, c, d: (a + b + c + d + 2) // 4

    own = _own_grid(w, h, phase)
    left_intent = _left_intent_grid(w, h, phase)
    r = np.zeros((h, w), np.int64)
    g = np.zeros((h, w), np.int64)
    b = np.zeros((h, w), np.int64)

    m_r = own == "R"
    m_b = own == "B"
    m_g = own == "G"

    g[m_r | m_b] = avg4(up, down, left, right)[m_r | m_b]
    diag = avg4(ul, ur, dl, dr)
    r[m_r] = src[m_r]
    b[m_r] = diag[m_r]
    b[m_b] = src[m_b]
    r[m_b] = diag[m_b]

    g[m_g] = src[m_g]
    horiz_is_r = (left_intent == "R") & m_g
    vert_is_r = m_g & ~horiz_is_r
    r[horiz_is_r] = avg2(left, right)[horiz_is_r]
    b[horiz_is_r] = avg2(up, down)[horiz_is_r]
    b[vert_is_r] = avg2(left, right)[vert_is_r]
    r[vert_is_r] = avg2(up, down)[vert_is_r]
    return np.stack([r, g, b], axis=-1)


def y8_from_rgb_np(rgb: np.ndarray, bits: int = 10) -> np.ndarray:
    """与金参考一致的固定亮度映射（截断 + 77/150/29 + 舍入 128）。"""
    s = bits - 8
    r8 = rgb[..., 0] >> s
    g8 = rgb[..., 1] >> s
    b8 = rgb[..., 2] >> s
    return ((77 * r8 + 150 * g8 + 29 * b8 + 128) >> 8).astype(np.uint8)


def to_y8(raw: np.ndarray, phase: int, bits: int = 10) -> np.ndarray:
    return y8_from_rgb_np(demosaic_bilinear_np(raw, phase), bits)


def load_frame_bin(path) -> dict:
    """读取帧合同 .bin，返回 header + (可选) 解出的样本矩阵。"""
    data = Path(path).read_bytes()
    info = FC.parse_frame(data)
    h = info["header"]
    out = {"header": h, "crc_ok": info["crc_ok"], "payload_ok": info["payload_ok"]}
    if info.get("pixels") is not None:
        out["raw"] = np.array(info["pixels"], dtype=np.uint16).reshape(h["height"], h["width"])
    return out


# --------------------------------------------------------------------------
# 自检：numpy 路径必须与纯 Python 金参考逐像素一致
# --------------------------------------------------------------------------

def selftest(verbose=True):
    res = []
    W, H = 16, 12
    for phase in range(4):
        raw_list = IA.synthetic_raw(W, H, "extreme")
        raw_np = np.array(raw_list, dtype=np.uint16).reshape(H, W)
        ref_rgb = IA.demosaic_bilinear(raw_list, W, H, phase, 10)
        ref = np.array(ref_rgb, dtype=np.int64).reshape(H, W, 3)
        got = demosaic_bilinear_np(raw_np, phase)
        res.append(("demosaic_matches_golden_phase%d" % phase, bool(np.array_equal(ref, got))))
        ref_y = np.array(IA.grayscale(raw_list, W, H, phase, 10), dtype=np.uint8).reshape(H, W)
        res.append(("y8_matches_golden_phase%d" % phase, bool(np.array_equal(ref_y, to_y8(raw_np, phase)))))

    # 逐字节往返：金参考 emit 的 packed 向量 → numpy 解包 → 原样本
    packed = FC.pack_raw10(IA.synthetic_raw(W, H, "ramp"))
    back = raw10_to_u16(packed, W, H)
    res.append(("raw10_unpack_matches", bool(np.array_equal(
        back, np.array(IA.synthetic_raw(W, H, "ramp"), dtype=np.uint16).reshape(H, W)))))

    # RGGB 与 BGGR 在极值图上必须不同（防止相位参数被忽略）
    a = to_y8(np.array(IA.synthetic_raw(W, H, "extreme"), dtype=np.uint16).reshape(H, W), 0)
    b = to_y8(np.array(IA.synthetic_raw(W, H, "extreme"), dtype=np.uint16).reshape(H, W), 1)
    res.append(("phase_parameter_effective", not np.array_equal(a, b)))

    # 退化尺寸：1x1 / 1xN / 4x1 不能崩，且与金参考一致
    for (w2, h2) in ((1, 1), (1, 4), (4, 1)):
        rl = [(i * 37 + 5) & 0x3FF for i in range(w2 * h2)]
        ref = np.array(IA.grayscale(rl, w2, h2, 0, 10), dtype=np.uint8).reshape(h2, w2)
        got = to_y8(np.array(rl, dtype=np.uint16).reshape(h2, w2), 0)
        res.append(("tiny_%dx%d_matches_golden" % (w2, h2), bool(np.array_equal(ref, got))))

    # width%4 != 0 的解包必须报错而不是静默截断
    try:
        raw10_to_u16(b"\x00" * 16, 6, 1)
        res.append(("width_not_mult4_rejected", False))
    except ValueError:
        res.append(("width_not_mult4_rejected", True))

    if verbose:
        for r in res:
            print("RK_FRAME_IO_SELFTEST", r)
    return res


if __name__ == "__main__":
    r = selftest()
    print("RK_FRAME_IO", "PASS" if all(x[1] for x in r) else "FAIL")
