#!/usr/bin/env python3
"""FPGA ↔ RK 非压缩帧 / 元数据接口合同：参考实现与共享向量（纯 Python）。

范围（边界）
------------
本模块只定义**非压缩**帧头 + 载荷 + 传输契约的部分：
  * 128B 小端帧头（字段/偏移固定），
  * RAW10 packed（MIPI 4px/5B）与 RAW10 unpacked uint16-LE 两种载荷布局，
  * CRC32（IEEE 反射 0xEDB88320，init/final xor 0xFFFFFFFF），
  * 合成 RAW10 共享向量（供 FPGA 侧打包 / RK 侧解析逐样本对照）。
**压缩码流字段（codec_id 取值、分块/字典/回退标记、块表）由 F2 负责**，本模块只保留占位字段并设 codec_id=0。

证据等级：这是**提案/草案**，未与队友冻结；代码经自检与往返对照，但不代表任一侧已实现。
"""

from __future__ import annotations

import argparse
import json
import struct
import zlib
from pathlib import Path

# --------------------------------------------------------------------------
# 常量
# --------------------------------------------------------------------------

MAGIC = b"R10F"
VERSION_MAJOR = 1
VERSION_MINOR = 0
HEADER_BYTES = 128

FMT_RAW10_PACKED = 0
FMT_RAW10_U16LE = 1
FMT_GRAY8 = 2
FMT_RGB565 = 3

CODEC_NONE = 0  # 压缩取值由 F2 定义

BAYER_PHASE = {"RGGB": 0, "BGGR": 1, "GRBG": 2, "GBRG": 3}
BAYER_PHASE_REV = {v: k for k, v in BAYER_PHASE.items()}

FLAG_BAD = 1 << 0        # 帧被拒/中止
FLAG_CROPPED = 1 << 1    # 使用了 ROI 裁剪
FLAG_PARTIAL = 1 << 2    # 载荷不完整（不应发布）
FLAG_OVERSIZE = 1 << 3   # 超过缓冲容量，已丢

# 帧头字段：(offset, name, struct_fmt) —— 全部小端
HEADER_FIELDS = [
    (0x00, "magic", "4s"),
    (0x04, "version_major", "<H"),
    (0x06, "version_minor", "<H"),
    (0x08, "header_bytes", "<H"),
    (0x0A, "camera_id", "<B"),
    (0x0B, "format_id", "<B"),
    (0x0C, "codec_id", "<I"),
    (0x10, "session_id", "<I"),
    (0x14, "capture_seq", "<I"),
    (0x18, "mode_id", "<I"),
    (0x1C, "width", "<H"),
    (0x1E, "height", "<H"),
    (0x20, "stride_bytes", "<I"),
    (0x24, "roi_x", "<H"),
    (0x26, "roi_y", "<H"),
    (0x28, "bayer_phase", "<B"),
    (0x29, "bayer_dir", "<B"),
    (0x2A, "valid_bits", "<B"),
    (0x2B, "packing", "<B"),
    (0x2C, "raw_bytes", "<I"),
    (0x30, "payload_bytes", "<I"),
    (0x34, "block_count", "<I"),
    (0x38, "exposure_code", "<I"),
    (0x3C, "gain_code", "<I"),
    (0x40, "sof_timestamp", "<Q"),
    (0x48, "timestamp_hz", "<I"),
    (0x4C, "flags", "<I"),
    (0x50, "raw_crc32", "<I"),
    (0x54, "payload_crc32", "<I"),
    (0x58, "header_crc32", "<I"),
]
HEADER_CRC_OFF = 0x58
RESERVED_RANGE = (0x5C, HEADER_BYTES)


# --------------------------------------------------------------------------
# CRC 与载荷编解码
# --------------------------------------------------------------------------

def crc32_ieee(data: bytes) -> int:
    """IEEE 反射 CRC32：poly 0xEDB88320，init/final xor 0xFFFFFFFF。

    check("123456789") == 0xCBF43926（见自检）。
    """
    return zlib.crc32(data) & 0xFFFFFFFF


def pack_raw10(pixels) -> bytes:
    """MIPI CSI-2 RAW10：每 4 个 10bit 样本压成 5 字节。

    字节序：P0[9:2] P1[9:2] P2[9:2] P3[9:2] {P3[1:0],P2[1:0],P1[1:0],P0[1:0]}
    要求 len(pixels) % 4 == 0。
    """
    if len(pixels) % 4:
        raise ValueError("RAW10 packing requires pixel count multiple of 4")
    out = bytearray()
    for i in range(0, len(pixels), 4):
        p = [pixels[i + k] & 0x3FF for k in range(4)]
        out += bytes([p[0] >> 2, p[1] >> 2, p[2] >> 2, p[3] >> 2])
        out.append(((p[3] & 3) << 6) | ((p[2] & 3) << 4) | ((p[1] & 3) << 2) | (p[0] & 3))
    return bytes(out)


def unpack_raw10(data: bytes, npix: int):
    """pack_raw10 的逆过程，返回 npix 个 10bit 样本。"""
    if npix % 4:
        raise ValueError("RAW10 unpack requires pixel count multiple of 4")
    need = npix // 4 * 5
    if len(data) < need:
        raise ValueError("payload too short")
    out = []
    for i in range(0, need, 5):
        b0, b1, b2, b3, b4 = data[i:i + 5]
        out += [(b0 << 2) | (b4 & 3), (b1 << 2) | ((b4 >> 2) & 3),
                (b2 << 2) | ((b4 >> 4) & 3), (b3 << 2) | ((b4 >> 6) & 3)]
    return out


def pack_u16le(pixels) -> bytes:
    return struct.pack("<%dH" % len(pixels), *[p & 0xFFFF for p in pixels])


def unpack_u16le(data: bytes, npix: int):
    return list(struct.unpack("<%dH" % npix, data[:npix * 2]))


def stride_for(format_id: int, width: int) -> int:
    if format_id == FMT_RAW10_PACKED:
        if width % 4:
            raise ValueError("packed RAW10 requires width multiple of 4")
        return width // 4 * 5
    if format_id == FMT_RAW10_U16LE:
        return width * 2
    if format_id == FMT_GRAY8:
        return width
    if format_id == FMT_RGB565:
        return width * 2
    raise ValueError("unknown format_id")


# --------------------------------------------------------------------------
# 帧头
# --------------------------------------------------------------------------

def pack_header(**kw) -> bytes:
    """按固定布局打包 128B 帧头，自动计算 raw/payload/header CRC。"""
    buf = bytearray(HEADER_BYTES)
    fields = {
        "magic": MAGIC, "version_major": VERSION_MAJOR, "version_minor": VERSION_MINOR,
        "header_bytes": HEADER_BYTES, "camera_id": 0, "format_id": FMT_RAW10_PACKED,
        "codec_id": CODEC_NONE, "session_id": 0, "capture_seq": 0, "mode_id": 0,
        "width": 0, "height": 0, "stride_bytes": 0, "roi_x": 0, "roi_y": 0,
        "bayer_phase": 0, "bayer_dir": 0, "valid_bits": 10, "packing": 0,
        "raw_bytes": 0, "payload_bytes": 0, "block_count": 1,
        "exposure_code": 0, "gain_code": 0, "sof_timestamp": 0, "timestamp_hz": 0,
        "flags": 0, "raw_crc32": 0, "payload_crc32": 0, "header_crc32": 0,
    }
    fields.update(kw)
    for off, name, fmt in HEADER_FIELDS:
        if name == "header_crc32":
            continue
        struct.pack_into(fmt, buf, off, fields[name])
    struct.pack_into("<I", buf, HEADER_CRC_OFF, 0)
    struct.pack_into("<I", buf, HEADER_CRC_OFF, crc32_ieee(bytes(buf)))
    return bytes(buf)


def unpack_header(buf: bytes) -> dict:
    if len(buf) < HEADER_BYTES:
        raise ValueError("header too short")
    out = {}
    for off, name, fmt in HEADER_FIELDS:
        (val,) = struct.unpack_from(fmt, buf, off)
        out[name] = val
    return out


def header_crc_ok(buf: bytes) -> bool:
    tmp = bytearray(buf[:HEADER_BYTES])
    stored = struct.unpack_from("<I", tmp, HEADER_CRC_OFF)[0]
    struct.pack_into("<I", tmp, HEADER_CRC_OFF, 0)
    return crc32_ieee(bytes(tmp)) == stored


# --------------------------------------------------------------------------
# 帧文件（帧头 + 载荷）
# --------------------------------------------------------------------------

def build_frame(pixels, width, height, format_id=FMT_RAW10_PACKED, phase="RGGB",
                camera_id=0, session_id=1, capture_seq=1, mode_id=1,
                exposure_code=0, gain_code=0, pad_to=32, flags=0) -> bytes:
    if format_id == FMT_RAW10_PACKED:
        payload = pack_raw10(pixels)
    elif format_id == FMT_RAW10_U16LE:
        payload = pack_u16le(pixels)
    else:
        raise ValueError("build_frame supports RAW10 packed / u16 only")
    raw_bytes = len(payload)
    pad = (-raw_bytes) % pad_to
    payload_padded = payload + b"\x00" * pad
    header = pack_header(
        camera_id=camera_id, format_id=format_id, session_id=session_id,
        capture_seq=capture_seq, mode_id=mode_id, width=width, height=height,
        stride_bytes=stride_for(format_id, width), roi_x=0, roi_y=0,
        bayer_phase=BAYER_PHASE[phase], bayer_dir=0, valid_bits=10,
        packing=0 if format_id == FMT_RAW10_PACKED else 1,
        raw_bytes=raw_bytes, payload_bytes=len(payload_padded), block_count=1,
        exposure_code=exposure_code, gain_code=gain_code, flags=flags,
        raw_crc32=crc32_ieee(payload), payload_crc32=crc32_ieee(payload_padded))
    return header + payload_padded


def parse_frame(data: bytes) -> dict:
    """解析帧并校验 magic/版本/几何/CRC；返回描述与（可选的）10bit 样本。

    **畸形输入不抛异常**，而是把对应标志置 False 并返回，便于上层统一处理。
    """
    res = {"geometry_ok": False, "crc_ok": False, "payload_ok": False,
           "pixels": None, "errors": []}
    if len(data) < HEADER_BYTES:
        res["errors"].append("frame shorter than header")
        return res
    h = unpack_header(data)
    res["header"] = h
    res["magic_ok"] = h["magic"] == MAGIC
    res["version_ok"] = (h["version_major"], h["version_minor"]) == (VERSION_MAJOR, VERSION_MINOR)
    res["header_bytes_ok"] = h["header_bytes"] == HEADER_BYTES
    res["header_crc_ok"] = header_crc_ok(data)
    if not res["header_crc_ok"]:
        res["errors"].append("header crc mismatch")
    try:
        expect_stride = stride_for(h["format_id"], h["width"])
    except Exception as exc:                      # 未知 format_id / width%4!=0
        res["errors"].append("stride: %s" % exc)
        return res
    res["geometry_ok"] = (h["width"] > 0 and h["height"] > 0
                          and h["stride_bytes"] == expect_stride)
    if not res["geometry_ok"]:
        res["errors"].append("geometry mismatch")
    raw_expected = h["stride_bytes"] * h["height"]
    if h["raw_bytes"] != raw_expected:
        res["errors"].append("raw_bytes %d != stride*height %d" % (h["raw_bytes"], raw_expected))
    if h["raw_bytes"] > h["payload_bytes"]:
        res["errors"].append("raw_bytes > payload_bytes")
    payload = data[HEADER_BYTES:HEADER_BYTES + h["payload_bytes"]]
    if len(payload) < h["payload_bytes"]:
        res["errors"].append("payload truncated")
        return res
    res["raw_crc_ok"] = crc32_ieee(payload[:h["raw_bytes"]]) == h["raw_crc32"]
    res["payload_crc_ok"] = crc32_ieee(payload) == h["payload_crc32"]
    res["crc_ok"] = res["raw_crc_ok"] and res["payload_crc_ok"]
    npix = h["width"] * h["height"]
    try:
        if h["format_id"] == FMT_RAW10_PACKED:
            res["pixels"] = unpack_raw10(payload[:h["raw_bytes"]], npix)
        elif h["format_id"] == FMT_RAW10_U16LE:
            res["pixels"] = unpack_u16le(payload[:h["raw_bytes"]], npix)
        else:
            res["errors"].append("format_id %d not decodable here" % h["format_id"])
            return res
    except Exception as exc:
        res["errors"].append("decode: %s" % exc)
        return res
    res["payload_ok"] = len(res["pixels"]) == npix
    return res


# --------------------------------------------------------------------------
# 合成向量
# --------------------------------------------------------------------------

def synthetic_pixels(width, height):
    """确定性合成 RAW10：2D 渐变 + 四角标记（0 / 1023 / 512 / 1）。"""
    if width <= 0 or height <= 0:
        return []
    px = [0] * (width * height)
    for y in range(height):
        for x in range(width):
            px[y * width + x] = (x * 3 + y * 5) & 0x3FF
    px[0] = 0
    px[width - 1] = 1023
    px[(height - 1) * width] = 512
    px[width * height - 1] = 1
    return px


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def crc32_rtl_style(data: bytes) -> int:
    """按现有 RTL 的写法（gray_frame_pack_guarded.v / ddr_display_read.v 的 crc_byte）
    逐步计算，用于证明「FPGA packer 的 CRC 定义」与合同一致。

    RTL: init 0xFFFFFFFF；每字节 v=c^b，8 次 v=(v>>1)^(v[0]?0xEDB88320:0)；最后取反。
    """
    c = 0xFFFFFFFF
    for b in data:
        v = c ^ b
        for _ in range(8):
            v = (v >> 1) ^ (0xEDB88320 if (v & 1) else 0)
        c = v
    return (~c) & 0xFFFFFFFF


def _selftest(verbose=True):
    res = []
    # CRC 标准检查值
    res.append(("crc32_check_123456789", crc32_ieee(b"123456789") == 0xCBF43926))
    # 合同 CRC == 现有 RTL CRC 定义（逐位复刻后与 zlib 比对）
    samples = [b"", b"123456789", bytes(range(256)), b"\x00" * 64,
               b"\xff" * 66, bytes((i * 37 + 11) & 0xFF for i in range(1000))]
    res.append(("crc32_matches_rtl_definition",
                all(crc32_ieee(s) == crc32_rtl_style(s) for s in samples)))
    # RAW10 打包往返（覆盖全部 10bit 值）
    allvals = [v & 0x3FF for v in range(0, 4096)]
    allvals = allvals[: (len(allvals) // 4) * 4]
    rt = unpack_raw10(pack_raw10(allvals), len(allvals)) == allvals
    res.append(("raw10_pack_roundtrip_allvalues", rt))
    # 帧头往返 + CRC
    hdr = pack_header(width=64, height=48, stride_bytes=stride_for(FMT_RAW10_PACKED, 64),
                      raw_bytes=3840, payload_bytes=3840)
    ok = (unpack_header(hdr)["header_bytes"] == HEADER_BYTES and header_crc_ok(hdr))
    res.append(("header_roundtrip_crc", ok))
    # 篡改检测
    bad = bytearray(hdr); bad[0x14] ^= 0xFF
    res.append(("header_tamper_detected", not header_crc_ok(bytes(bad))))

    # 畸形输入必须返回 flags 而不是抛异常
    px = synthetic_pixels(8, 4)
    good = build_frame(px, 8, 4)
    def safe_parse(b):
        try:
            return parse_frame(b)
        except Exception:
            return None
    r1 = safe_parse(good[:100])                       # 截断
    res.append(("malformed_truncated_returns_flags",
                r1 is not None and not r1["crc_ok"] and not r1["payload_ok"]))
    h_bad = bytearray(good); struct.pack_into("<B", h_bad, 0x0B, 9)   # 未知 format_id
    struct.pack_into("<I", h_bad, HEADER_CRC_OFF, 0)
    struct.pack_into("<I", h_bad, HEADER_CRC_OFF, crc32_ieee(bytes(h_bad)))
    r2 = safe_parse(bytes(h_bad))
    res.append(("malformed_format_id_returns_flags",
                r2 is not None and not r2["geometry_ok"] and bool(r2["errors"])))
    h_bad2 = bytearray(good); struct.pack_into("<H", h_bad2, 0x1C, 7)  # width=7 非 4 倍数
    struct.pack_into("<I", h_bad2, HEADER_CRC_OFF, 0)
    struct.pack_into("<I", h_bad2, HEADER_CRC_OFF, crc32_ieee(bytes(h_bad2)))
    r3 = safe_parse(bytes(h_bad2))
    res.append(("malformed_width_not_mult4_returns_flags",
                r3 is not None and not r3["geometry_ok"] and bool(r3["errors"])))
    res.append(("wellformed_still_passes",
                safe_parse(good)["crc_ok"] and safe_parse(good)["payload_ok"]))
    if verbose:
        for r in res:
            print("FRAME_CONTRACT_SELFTEST", r)
    return res


def main():
    ap = argparse.ArgumentParser(description="非压缩帧/元数据接口合同参考实现")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--emit", metavar="DIR", help="生成共享向量到目录")
    ap.add_argument("--width", type=int, default=64)
    ap.add_argument("--height", type=int, default=48)
    ap.add_argument("--phase", default="RGGB", choices=list(BAYER_PHASE))
    ap.add_argument("--verify", metavar="FILE")
    a = ap.parse_args()

    if a.selftest:
        res = _selftest()
        print("FRAME_CONTRACT", "PASS" if all(r[1] for r in res) else "FAIL")
        return

    if a.verify:
        data = Path(a.verify).read_bytes()
        info = parse_frame(data)
        print("FRAME_PARSE", json.dumps(
            {k: v for k, v in info.items() if k not in ("header", "pixels")}, ensure_ascii=False))
        if "header" in info:
            print("FRAME_HEADER", json.dumps(
                {k: (v.decode() if isinstance(v, bytes) else v) for k, v in info["header"].items()},
                ensure_ascii=False))
        ok = bool(info.get("magic_ok") and info.get("version_ok") and info.get("header_bytes_ok")
                  and info.get("header_crc_ok") and info.get("geometry_ok")
                  and info.get("crc_ok") and info.get("payload_ok"))
        print("FRAME_VERIFY", "PASS" if ok else "FAIL")
        if not ok:
            raise SystemExit(1)
        return

    if a.emit:
        out = Path(a.emit)
        out.mkdir(parents=True, exist_ok=True)
        px = synthetic_pixels(a.width, a.height)
        for name, fmt in (("packed", FMT_RAW10_PACKED), ("u16le", FMT_RAW10_U16LE)):
            blob = build_frame(px, a.width, a.height, format_id=fmt, phase=a.phase)
            f = out / ("frame_%dx%d_%s.bin" % (a.width, a.height, name))
            f.write_bytes(blob)
            info = parse_frame(blob)
            assert info["crc_ok"] and info["payload_ok"] and info["pixels"] == px, name
            print("FRAME_EMITTED", f, len(blob))
        print("FRAME_EMIT_DONE", out)
        return

    ap.print_help()


if __name__ == "__main__":
    main()
