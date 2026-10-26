#!/usr/bin/env python3
"""三种硬件预处理算法的独立软件金参考（纯 Python，仅标准库）。

覆盖赛题 R2 的三条分支：灰度、3x3 高斯、前帧 CDF-min 直方图均衡。

语义来源（必须与 RTL 冻结一致，本文把文档里没写死的舍入/边界**显式定义**）：
  * docs/双目首版_整链路方案与团队交付.md §3：
      - 灰度：Bayer 3x3 双线性去马赛克 + 复制边界 -> 每分量截高位到 8bit
              -> Y8 = (77R8 + 150G8 + 29B8 + 128) >> 8
      - 高斯：Y8 上 3x3 [1 2 1; 2 4 2; 1 2 1]，(sum+8)>>4，复制边界，尺寸不变
      - 均衡：最近一个完整有效**前帧**的 CDF-min LUT；首帧恒等；坏帧不发布新 LUT；
              四像素并行分 4 个直方图 bank，帧后合并
  * fpga/csi2/src/mem/bayer565_capture.v：相位定义 Phase0=RGGB,1=BGGR,2=GRBG,3=GBRG
    （按**接收像素/行顺序**）

本文件显式定义、且 RTL/队友必须复用的口径：
  * 平均的舍入：`(sum + n//2) // n`（n=2 即 (sum+1)>>1，n=4 即 (sum+2)>>2）。
  * 8bit 截取：`>> (BITS-8)`（RAW10 -> >>2），**截断不是四舍五入**。
  * CDF-min 分母为 0（常量图）时：恒等 LUT。
  * LUT 值：`clamp(round((cdf[v]-cdf_min)*255/den), 0, 255)`，round 为四舍五入。
  * 灰度位宽：默认 RAW10（BITS=10）；RAW12 封存分支可传 BITS=12。

边界：这是**提案式金参考**，不是 RTL、不是实拍结果；未与写 RTL 的线程冻结语义。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

# 相位 -> (TL, TR, BL, BR) 颜色，按接收像素/行顺序
PHASE_COLORS = {
    0: "RGGB", 1: "BGGR", 2: "GRBG", 3: "GBRG",
}
PHASE_ID = {v: k for k, v in PHASE_COLORS.items()}
_LUT4 = {  # 每相位的 4 个 CFA 位置颜色
    0: "R G G B", 1: "B G G R", 2: "G R B G", 3: "G B R G",
}


def cfa_color(y: int, x: int, phase: int) -> str:
    """返回 (y,x) 处的 CFA 颜色 'R'/'G'/'B'。"""
    tl, tr, bl, br = _LUT4[phase].split()
    if y % 2 == 0:
        return tl if x % 2 == 0 else tr
    return bl if x % 2 == 0 else br


def _clamp_xy(x, y, width, height):
    return min(max(x, 0), width - 1), min(max(y, 0), height - 1)


def _clamp_color(x, y, width, height, dx, dy):
    """把邻居 (x+dx, y+dy) 夹回图像，并**保持 CFA 奇偶**，使颜色与目标一致。

    这是"复制边界"的正确实现：直接夹取坐标会在边框取到异色像素
    （角点整列/整行通道错），保持奇偶则等价于在同色格点上复制。
    若图像某一维只有 1 像素、无处可挪，则退化为该中心像素。
    """
    tx, ty = x + dx, y + dy
    cx = min(max(tx, 0), width - 1)
    if cx % 2 != tx % 2:
        if cx + 1 <= width - 1:
            cx += 1
        elif cx - 1 >= 0:
            cx -= 1
        else:
            return x, y
    cy = min(max(ty, 0), height - 1)
    if cy % 2 != ty % 2:
        if cy + 1 <= height - 1:
            cy += 1
        elif cy - 1 >= 0:
            cy -= 1
        else:
            return x, y
    return cx, cy


def _avg(values):
    n = len(values)
    return (sum(values) + n // 2) // n


def demosaic_bilinear(raw, width, height, phase, bits=10):
    """3x3 双线性去马赛克 + 复制边界。raw 为行主序样本列表。

    返回 [(r,g,b)]，分量位宽 = bits。
    """
    if len(raw) != width * height:
        raise ValueError("raw size mismatch")
    out = []
    at = lambda x, y: raw[y * width + x]

    def nb(x, y, dx, dy):
        cx, cy = _clamp_color(x, y, width, height, dx, dy)
        return at(cx, cy)

    for y in range(height):
        for x in range(width):
            own = cfa_color(y, x, phase)
            if own == "G":
                # 方向用**未经夹取**的奇偶判定（x=0 也正确）
                horiz = cfa_color(y, x - 1, phase)
                if horiz == "R":
                    r = _avg([nb(x, y, -1, 0), nb(x, y, 1, 0)])
                    b = _avg([nb(x, y, 0, -1), nb(x, y, 0, 1)])
                else:
                    b = _avg([nb(x, y, -1, 0), nb(x, y, 1, 0)])
                    r = _avg([nb(x, y, 0, -1), nb(x, y, 0, 1)])
                g = at(x, y)
            else:
                g = _avg([nb(x, y, 0, -1), nb(x, y, 0, 1),
                          nb(x, y, -1, 0), nb(x, y, 1, 0)])
                diag = _avg([nb(x, y, -1, -1), nb(x, y, 1, -1),
                             nb(x, y, -1, 1), nb(x, y, 1, 1)])
                if own == "R":
                    r, b = at(x, y), diag
                else:
                    b, r = at(x, y), diag
            out.append((r, g, b))
    return out


def y8_from_rgb(rgb, bits=10):
    """固定亮度映射：R8=r>>(bits-8) 截断，(77R+150G+29B+128)>>8。"""
    shift = bits - 8
    out = []
    for r, g, b in rgb:
        r8, g8, b8 = r >> shift, g >> shift, b >> shift
        out.append((77 * r8 + 150 * g8 + 29 * b8 + 128) >> 8)
    return out


def grayscale(raw, width, height, phase, bits=10):
    return y8_from_rgb(demosaic_bilinear(raw, width, height, phase, bits), bits)


def gaussian3x3(img, width, height):
    """3x3 [1 2 1; 2 4 2; 1 2 1]，(sum+8)>>4，复制边界。"""
    if len(img) != width * height:
        raise ValueError("image size mismatch")
    at = lambda x, y: img[min(max(y, 0), height - 1) * width + min(max(x, 0), width - 1)]
    out = []
    for y in range(height):
        for x in range(width):
            s = (at(x - 1, y - 1) + 2 * at(x, y - 1) + at(x + 1, y - 1)
                 + 2 * at(x - 1, y) + 4 * at(x, y) + 2 * at(x + 1, y)
                 + at(x - 1, y + 1) + 2 * at(x, y + 1) + at(x + 1, y + 1))
            out.append((s + 8) >> 4)
    return out


def histogram(img):
    h = [0] * 256
    for v in img:
        h[v & 0xFF] += 1
    return h


def banks4_histogram(img, width=0):
    """模拟 4 像素/拍、4 bank：第 i 个像素进第 i%4 个 bank，帧后合并。

    同一拍多个像素落在同一 bin 也不会丢计数（文档要求）。

    ⚠️ 口径：本函数与 `histogram` 在数学上**恒等**（每个像素只进一个 bank 再求和）。
    它记录的是**语义要求**，**不是对真实 4-bank 硬件的验证**；硬件 bank 冲突语义
    需由 RTL 提供后另做对拍。`width` 形参保留仅为调用处可读性。
    """
    banks = [[0] * 256 for _ in range(4)]
    for i, v in enumerate(img):
        banks[i % 4][v & 0xFF] += 1
    merged = [0] * 256
    for b in banks:
        for i in range(256):
            merged[i] += b[i]
    return merged


def banks4_histogram_buggy(img):
    """**错误的** 4-bank 实现：同一拍内同 bin 只记一次（会丢计数）。

    仅用于负对照：证明 `banks4_histogram` 的"同 bin 不丢计数"不是恒真命题。
    """
    banks = [[0] * 256 for _ in range(4)]
    for i in range(0, len(img), 4):
        seen = set()
        for j, v in enumerate(img[i:i + 4]):
            if v in seen:
                continue
            seen.add(v)
            banks[j % 4][v & 0xFF] += 1
    merged = [0] * 256
    for b in banks:
        for i in range(256):
            merged[i] += b[i]
    return merged


def cdf_min_lut(hist, npix):
    """CDF-min 均衡 LUT；分母 0 时返回恒等 LUT。"""
    cdf_min = None
    acc = 0
    for v in range(256):
        acc += hist[v]
        if cdf_min is None and hist[v] > 0:
            cdf_min = acc
    den = npix - (cdf_min or 0)
    if cdf_min is None or den <= 0:
        return list(range(256))
    lut = []
    for v in range(256):
        acc_v = sum(hist[:v + 1])
        num = (acc_v - cdf_min) * 255
        val = (num + den // 2) // den
        lut.append(0 if val < 0 else (255 if val > 255 else val))
    return lut


def apply_lut(img, lut):
    return [lut[v] for v in img]


class HeLut:
    """前帧 CDF-min 均衡状态机（软件参考）。

    - 首帧用恒等 LUT；
    - 当前帧边统计边用旧 LUT 输出；
    - 帧结束若该帧有效则用其直方图发布新 LUT，否则保持旧 LUT 并计数。
    """

    def __init__(self):
        self.lut = list(range(256))
        self.lut_source_seq = 0
        self.published = 0
        self.rejected = 0

    def begin_frame(self):
        return list(self.lut)  # 本帧输出用哪个 LUT

    def end_frame(self, img, valid):
        if not valid:
            self.rejected += 1
            return False
        self.lut = cdf_min_lut(banks4_histogram(img, 0), len(img))
        self.published += 1
        self.lut_source_seq = self.published
        return True


# --------------------------------------------------------------------------
# 自检
# --------------------------------------------------------------------------

def _color_only_raw(width, height, phase, r, g, b):
    """构造每个 CFA 位置只放对应颜色的样本（用于验证相位与邻居选择）。"""
    raw = []
    for y in range(height):
        for x in range(width):
            c = cfa_color(y, x, phase)
            raw.append({"R": r, "G": g, "B": b}[c])
    return raw


def selftest(verbose=True):
    res = []
    # 1) 纯色 CFA：去马赛克必须**全图逐点**精确还原 (r,g,b)
    #    （保色夹取后，边框/角点不再渗入异色）
    ok = True
    w, h = 8, 6
    for phase in range(4):
        raw = _color_only_raw(w, h, phase, 200, 400, 700)
        got = demosaic_bilinear(raw, w, h, phase)
        if any(px != (200, 400, 700) for px in got):
            ok = False
    res.append(("demosaic_coloronly_exact_all", ok))

    # 1b) 显式钉住角点行为（(0,0) 相位0）：四对角都落到 (1,1) 的 B，四正交都落到 G
    raw = _color_only_raw(w, h, 0, 200, 400, 700)
    corner = demosaic_bilinear(raw, w, h, 0)[0]
    res.append(("border_color_aware_corner", corner == (200, 400, 700)))

    # 1c) 左边界绿色像素的方向不能再反向（相位 1/2 曾整列错）
    raw = [0] * (w * h)
    for yy in range(h):
        for xx in range(w):
            c = cfa_color(yy, xx, 1)          # BGGR
            raw[yy * w + xx] = {"R": 900, "G": 100, "B": 50}[c]
    row1 = demosaic_bilinear(raw, w, h, 1)[1 * w + 0]   # (x=0,y=1) 是绿色
    res.append(("left_green_direction_phase1", row1 == (900, 100, 50)))

    # 2) 平坦灰 CFA（所有样本相同）-> r=g=b
    raw = [511] * (8 * 6)
    got = demosaic_bilinear(raw, 8, 6, 0)
    res.append(("demosaic_flat_is_neutral", all(px == (511, 511, 511) for px in got)))

    # 3) Y8 权重和 = 256；满量程应为 255（10bit 输入 1023 与 12bit 输入 4095）
    res.append(("y8_weights_sum_256", 77 + 150 + 29 == 256))
    res.append(("y8_white_255_raw10", y8_from_rgb([(1023, 1023, 1023)], 10)[0] == 255))
    res.append(("y8_white_255_raw12", y8_from_rgb([(4095, 4095, 4095)], 12)[0] == 255))
    res.append(("y8_black_0", y8_from_rgb([(0, 0, 0)], 10)[0] == 0))

    # 3b) 退化尺寸不能崩（1x1 / 1xN / 4x1）
    tiny_ok = True
    try:
        for (w2, h2) in ((1, 1), (1, 4), (4, 1)):
            rl = [(i * 37 + 5) & 0x3FF for i in range(w2 * h2)]
            rgb = demosaic_bilinear(rl, w2, h2, 0)
            y = y8_from_rgb(rgb, 10)
            g = gaussian3x3(y, w2, h2)
            if len(rgb) != w2 * h2 or len(y) != w2 * h2 or len(g) != w2 * h2:
                tiny_ok = False
    except Exception:
        tiny_ok = False
    res.append(("tiny_geometries_ok", tiny_ok))
    res.append(("tiny_1x1_shape", demosaic_bilinear([500], 1, 1, 0) == [(500, 500, 500)]))

    # 4) 高斯：常量保持；内部冲激响应 = 权重/16
    flat = [128] * (9 * 9)
    res.append(("gaussian_constant_identity", gaussian3x3(flat, 9, 9) == flat))
    imp = [0] * (9 * 9)
    imp[4 * 9 + 4] = 16
    gi = gaussian3x3(imp, 9, 9)
    # 冲激 16 经权重 w 后为 (w*16+8)>>4：中心 w=4 -> 4，正交 w=2 -> 2，对角 w=1 -> 1
    expect = {(4, 4): 4, (3, 4): 2, (5, 4): 2, (4, 3): 2, (4, 5): 2,
              (3, 3): 1, (5, 3): 1, (3, 5): 1, (5, 5): 1}
    res.append(("gaussian_impulse_kernel", all(
        gi[y * 9 + x] == v for (x, y), v in expect.items())))
    res.append(("gaussian_impulse_energy_conserved", sum(gi) == 16))

    # 5) 4 bank 直方图 == 直接直方图（同 bin 不丢计数），并用错误实现做负对照
    img = [7] * 64                       # 全部同 bin（最坏的同拍冲突）
    res.append(("banks4_no_lost_counts", banks4_histogram(img, 8) == histogram(img)))
    res.append(("banks4_buggy_detected", banks4_histogram_buggy(img) != histogram(img)))
    img2 = [0, 1, 2, 3] * 16             # 同拍不同 bin
    res.append(("banks4_distinct_bins", banks4_histogram(img2, 8) == histogram(img2)))

    # 6) 常量图 -> 分母 0 -> 恒等 LUT
    res.append(("cdf_min_denominator_zero_identity",
                cdf_min_lut(histogram([7] * 64), 64) == list(range(256))))

    # 7) 前帧语义：首帧恒等；第二帧用第一帧的 LUT；坏帧不发布
    he = HeLut()
    first = he.begin_frame()
    res.append(("he_first_frame_identity", first == list(range(256))))
    ramp = [(i * 4) & 0xFF for i in range(64)]
    he.end_frame(ramp, valid=True)
    lut_after_1 = list(he.lut)
    res.append(("he_publishes_on_valid", he.published == 1 and lut_after_1 != list(range(256))))
    before = list(he.lut)
    he.end_frame([0] * 64, valid=False)
    res.append(("he_bad_frame_keeps_lut", he.lut == before and he.rejected == 1))

    # 8) LUT 单调不减
    res.append(("lut_monotonic", all(lut_after_1[i] <= lut_after_1[i + 1] for i in range(255))))

    if verbose:
        for r in res:
            print("IMG_ALGO_SELFTEST", r)
    return res


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def synthetic_raw(width, height, kind):
    if kind == "flat":
        return [128] * (width * height)
    if kind == "ramp":
        return [(y * 7 + x * 3) & 0x3FF for y in range(height) for x in range(width)]
    if kind == "impulse":
        raw = [0] * (width * height)
        raw[(height // 2) * width + width // 2] = 1023
        return raw
    if kind == "checker":
        return [1023 if (x + y) % 2 else 0 for y in range(height) for x in range(width)]
    if kind == "extreme":
        raw = [(x * 41 + y * 17) & 0x3FF for y in range(height) for x in range(width)]
        raw[0], raw[-1] = 0, 1023
        return raw
    raise ValueError("unknown kind")


def main():
    ap = argparse.ArgumentParser(description="三算法独立金参考")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        res = selftest()
        print("IMG_ALGO", "PASS" if all(r[1] for r in res) else "FAIL")
        return
    ap.print_help()


if __name__ == "__main__":
    main()
