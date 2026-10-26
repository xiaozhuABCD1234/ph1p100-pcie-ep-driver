#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T11.rk-align-raw：离线**严格** R10F v1 非压缩帧校验 / 样例生成（与冻结合同逐字段一致）

用途
----
1) `check`：对一份 R10F 传输内容（= 主机槽内 hdr_bytes 偏移起、长度 dma_bytes 的那段，
   即 128B 头 + packed 有效载荷 + 4KiB 对齐尾零）做**严格**校验。任何一项不符即拒绝
   （退出码 1），绝不做「夹紧后当成功」：
     · magic "R10F" / version 1.0 / header_bytes=128；
     · format_id=0（非压缩）、codec_id=0（BPR1 是别的 codec，不在这里）、
       valid_bits=10、packing=0；
     · 几何自洽：width%4==0、stride==width*5/4、raw==payload==stride*height；
     · 长度精确：文件长 == ceil((128+payload)/4096)*4096（三层长度分别命名）；
     · flags 只允许 0 或 2（CROPPED）；BAD|PARTIAL|OVERSIZE 一律拒绝；
     · reserved 0x5C..0x7F 全 0；尾零（content_bytes 之后）全 0；
     · raw_crc32 == payload_crc32 == CRC32(packed 载荷)；
       header_crc32 == CRC32(头 128B，其中 0x58..0x5B 先清 0)；
       CRC32 IEEE reflected poly 0xEDB88320、init/final 全 1；
     · 可选 --session / --width/--height：与授权值不符即拒绝。
2) `gen`：按同样规则生成样例（小几何与 1080p 预设；像素模式 = 合同验收向量的
   标记组 [0,1,1022,1023] 循环）。用相同参数生成的结果与
   `contract/acceptance-vectors.json` 的 `artifacts/r10f-*-synthetic.dma` **逐字节相同**
   （SHA256 由 `selftest` 复算核对）。
3) `selftest`：复算合同验收向量（SHA256 / 尺寸 / CRC / 逐样本组 / 全部 frame_mutations
   负例），并自检 gen 的可复现性。输出 CONTRACT_VECTORS_OK / FAIL。

边界：本工具只做**离线字节级**合同校验；它不证明 RTL、驱动、板级或真实相机几何。
"""
import argparse
import binascii
import hashlib
import json
import struct
import sys
from pathlib import Path

sys.dont_write_bytecode = True

# ---------------------------------------------------------------------------
# 头部字段表（与 references/local-inputs/parallel/frame/frame_contract.py 一致，全小端）
# ---------------------------------------------------------------------------
MAGIC = b"R10F"
VERSION_MAJOR = 1
VERSION_MINOR = 0
HEADER_BYTES = 128
FMT_RAW10_PACKED = 0
CODEC_NONE = 0
VALID_BITS = 10
FLAG_BAD = 1 << 0
FLAG_CROPPED = 1 << 1
FLAG_PARTIAL = 1 << 2
FLAG_OVERSIZE = 1 << 3

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


def crc32_ieee(data: bytes) -> int:
    """CRC32 IEEE reflected（poly 0xEDB88320、init/final 全 1）——与合同 crc32_rtl_style 一致"""
    return binascii.crc32(data) & 0xFFFFFFFF


def unpack_header(buf: bytes) -> dict:
    if len(buf) < HEADER_BYTES:
        raise ValueError("header too short")
    out = {}
    for off, name, fmt in HEADER_FIELDS:
        (val,) = struct.unpack_from(fmt, buf, off)
        out[name] = val
    return out


def pack_header(**kw) -> bytes:
    buf = bytearray(HEADER_BYTES)
    fields = {
        "magic": MAGIC, "version_major": VERSION_MAJOR, "version_minor": VERSION_MINOR,
        "header_bytes": HEADER_BYTES, "camera_id": 0, "format_id": FMT_RAW10_PACKED,
        "codec_id": CODEC_NONE, "session_id": 0, "capture_seq": 0, "mode_id": 0,
        "width": 0, "height": 0, "stride_bytes": 0, "roi_x": 0, "roi_y": 0,
        "bayer_phase": 0, "bayer_dir": 0, "valid_bits": VALID_BITS, "packing": 0,
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


def pack_raw10_group(samples) -> bytes:
    """4 个 10bit 样本 → 5 字节（P0[9:2],P1[9:2],P2[9:2],P3[9:2],{P3,P2,P1,P0}低2bit）"""
    if len(samples) != 4:
        raise ValueError("need exactly 4 samples")
    p = [s & 0x3FF for s in samples]
    return bytes([p[0] >> 2, p[1] >> 2, p[2] >> 2, p[3] >> 2,
                  ((p[3] & 3) << 6) | ((p[2] & 3) << 4) | ((p[1] & 3) << 2) | (p[0] & 3)])


def marker_payload(npix: int) -> bytes:
    """合同验收向量的像素模式：每组（4 像素）重复 [0, 1, 1022, 1023]"""
    if npix % 4:
        raise ValueError("pixel count must be multiple of 4")
    grp = pack_raw10_group([0, 1, 1022, 1023])
    return grp * (npix // 4)


def dma_len(payload_bytes: int) -> int:
    return (HEADER_BYTES + payload_bytes + 4095) // 4096 * 4096


def build_frame(width: int, height: int, session_id: int = 0x20261017, capture_seq: int = 1,
                roi_x: int = 0, roi_y: int = 0, bayer_phase: int = 3, camera_id: int = 0,
                mode_id: int = 1, flags=None, payload=None) -> bytes:
    """生成一份严格合规的 R10F 非压缩帧（默认标记组像素；flags 缺省按 ROI 自动置 2/0）"""
    if width % 4:
        raise ValueError("packed RAW10 requires width multiple of 4")
    stride = width // 4 * 5
    npix = width * height
    if payload is None:
        payload = marker_payload(npix)
    if len(payload) != stride * height:
        raise ValueError("payload length != stride*height")
    if flags is None:
        flags = FLAG_CROPPED if (roi_x or roi_y) else 0
    pcrc = crc32_ieee(payload)
    header = pack_header(
        camera_id=camera_id, session_id=session_id, capture_seq=capture_seq, mode_id=mode_id,
        width=width, height=height, stride_bytes=stride, roi_x=roi_x, roi_y=roi_y,
        bayer_phase=bayer_phase & 3, bayer_dir=0, raw_bytes=len(payload),
        payload_bytes=len(payload), block_count=1, flags=flags,
        raw_crc32=pcrc, payload_crc32=pcrc)
    content = header + payload
    tail = b"\x00" * (dma_len(len(payload)) - len(content))
    return content + tail


def strict_validate(data: bytes, expect_width=None, expect_height=None, expect_session=None):
    """严格校验；返回 (ok, errors, info)。任何一项不符 → ok=False（不做夹紧/容错）。"""
    errs = []
    info = {}
    if len(data) < HEADER_BYTES:
        return False, ["frame shorter than 128B header"], info
    h = unpack_header(data)
    info["header"] = h
    if h["magic"] != MAGIC:
        errs.append("bad magic %r" % (h["magic"],))
    if (h["version_major"], h["version_minor"]) != (VERSION_MAJOR, VERSION_MINOR):
        errs.append("version %d.%d != 1.0" % (h["version_major"], h["version_minor"]))
    if h["header_bytes"] != HEADER_BYTES:
        errs.append("header_bytes %d != 128" % h["header_bytes"])
    if h["format_id"] != FMT_RAW10_PACKED:
        errs.append("format_id %d != 0 (RAW10 packed)" % h["format_id"])
    if h["codec_id"] != CODEC_NONE:
        errs.append("codec_id %d unknown (compression codecs rejected here)" % h["codec_id"])
    if h["valid_bits"] != VALID_BITS or h["packing"] != 0:
        errs.append("valid_bits=%d packing=%d (expect 10 / 0)" % (h["valid_bits"], h["packing"]))
    if h["width"] == 0 or h["height"] == 0 or h["width"] % 4:
        errs.append("geometry %dx%d invalid (width must be nonzero multiple of 4)" % (h["width"], h["height"]))
    expect_stride = h["width"] // 4 * 5 if h["width"] % 4 == 0 else -1
    if h["stride_bytes"] != expect_stride:
        errs.append("stride %d != width*5/4 %d" % (h["stride_bytes"], expect_stride))
    if h["raw_bytes"] != h["payload_bytes"]:
        errs.append("raw_bytes %d != payload_bytes %d" % (h["raw_bytes"], h["payload_bytes"]))
    if h["payload_bytes"] != h["stride_bytes"] * h["height"]:
        errs.append("payload %d != stride*height %d" % (h["payload_bytes"], h["stride_bytes"] * h["height"]))
    bad_flags = h["flags"] & (FLAG_BAD | FLAG_PARTIAL | FLAG_OVERSIZE)
    if bad_flags or (h["flags"] & ~FLAG_CROPPED):
        errs.append("flags 0x%x not publishable (only 0/2=CROPPED allowed)" % h["flags"])
    if h["block_count"] < 1:
        errs.append("block_count %d < 1" % h["block_count"])
    content = HEADER_BYTES + h["payload_bytes"]
    want_len = dma_len(h["payload_bytes"])
    if len(data) != want_len:
        errs.append("frame length %d != dma_bytes %d" % (len(data), want_len))
    if len(data) >= content and any(data[content:]):
        errs.append("transport tail not all zero")
    if any(data[RESERVED_RANGE[0]:RESERVED_RANGE[1]]):
        errs.append("reserved 0x5C..0x7F not zero")
    if expect_width is not None and h["width"] != expect_width:
        errs.append("width %d != expected %d" % (h["width"], expect_width))
    if expect_height is not None and h["height"] != expect_height:
        errs.append("height %d != expected %d" % (h["height"], expect_height))
    if expect_session is not None and h["session_id"] != expect_session:
        errs.append("session 0x%08x != expected 0x%08x" % (h["session_id"], expect_session))
    payload = data[HEADER_BYTES:content] if len(data) >= content else b""
    if len(payload) == h["payload_bytes"]:
        pcrc = crc32_ieee(payload)
        info["payload_crc_calc"] = pcrc
        if pcrc != h["payload_crc32"]:
            errs.append("payload_crc32 0x%08x != calc 0x%08x" % (h["payload_crc32"], pcrc))
        if pcrc != h["raw_crc32"]:
            errs.append("raw_crc32 0x%08x != payload_crc32 0x%08x (non-compressed must be equal)"
                        % (h["raw_crc32"], h["payload_crc32"]))
    else:
        errs.append("payload truncated/short")
    hdr_copy = bytearray(data[:HEADER_BYTES])
    stored = struct.unpack_from("<I", hdr_copy, HEADER_CRC_OFF)[0]
    struct.pack_into("<I", hdr_copy, HEADER_CRC_OFF, 0)
    hcrc = crc32_ieee(bytes(hdr_copy))
    info["header_crc_calc"] = hcrc
    if hcrc != stored:
        errs.append("header_crc32 0x%08x != calc 0x%08x" % (stored, hcrc))
    return (len(errs) == 0), errs, info


def mutate(data: bytes, operation: str) -> bytes:
    """合同 frame_mutations 的等价实现（负例自检用）"""
    b = bytearray(data)

    def refresh():
        struct.pack_into("<I", b, HEADER_CRC_OFF, 0)
        struct.pack_into("<I", b, HEADER_CRC_OFF, crc32_ieee(bytes(b[:HEADER_BYTES])))

    if operation == "none":
        pass
    elif operation == "drop_last_byte":
        b = b[:-1]
    elif operation == "tail_nonzero":
        b[-1] = 1
    elif operation == "payload_flip":
        b[HEADER_BYTES] ^= 1
    elif operation == "header_flip":
        b[0x18] ^= 1
    elif operation == "set_codec_1_recrc":
        struct.pack_into("<I", b, 0x0C, 1); refresh()
    elif operation == "set_bad_recrc":
        struct.pack_into("<I", b, 0x4C, 1); refresh()
    elif operation == "set_partial_recrc":
        struct.pack_into("<I", b, 0x4C, 4); refresh()
    elif operation == "set_oversize_recrc":
        struct.pack_into("<I", b, 0x4C, 8); refresh()
    elif operation == "set_length_recrc":
        raw = struct.unpack_from("<I", b, 0x30)[0]
        struct.pack_into("<I", b, 0x30, raw + 1); refresh()
    else:
        raise ValueError("unknown mutation %s" % operation)
    return bytes(b)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def cmd_check(args):
    data = Path(args.file).read_bytes()
    ok, errs, info = strict_validate(
        data, expect_width=args.width, expect_height=args.height,
        expect_session=(int(args.session, 0) if args.session is not None else None))
    out = {
        "file": str(args.file),
        "sha256": hashlib.sha256(data).hexdigest(),
        "size": len(data),
        "ok": ok,
        "errors": errs,
        "header": {k: (v.decode() if isinstance(v, bytes) else v) for k, v in info.get("header", {}).items()},
        "header_crc_calc": info.get("header_crc_calc"),
        "payload_crc_calc": info.get("payload_crc_calc"),
        "marker": "R10F_STRICT_OK" if ok else "R10F_STRICT_FAIL",
    }
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({k: out[k] for k in ("file", "size", "ok", "marker")}, ensure_ascii=False))
    for e in errs:
        print("  - %s" % e)
    return 0 if ok else 1


PRESETS = {  # (width, height, roi_x, roi_y, flags)
    "small": (8, 4, 0, 0, 0),
    "1080p": (1920, 1080, 960, 540, FLAG_CROPPED),
}


def cmd_gen(args):
    w, h, rx, ry, fl = PRESETS[args.preset]
    if args.width:
        w = args.width
    if args.height:
        h = args.height
    frame = build_frame(w, h, session_id=int(args.session, 0), capture_seq=args.seq,
                        roi_x=rx, roi_y=ry, bayer_phase=args.phase, camera_id=args.camera,
                        mode_id=args.mode, flags=fl)
    Path(args.out).write_bytes(frame)
    ok, errs, _ = strict_validate(frame)
    assert ok, errs
    print(json.dumps({"out": str(args.out), "size": len(frame),
                      "sha256": hashlib.sha256(frame).hexdigest(),
                      "marker": "R10F_GEN_OK"}, ensure_ascii=False))
    return 0


def cmd_selftest(args):
    run = Path(args.run_dir).resolve()
    vectors = json.loads((run / "contract" / "acceptance-vectors.json").read_text(encoding="utf-8"))
    results = []

    def chk(name, cond):
        results.append({"id": name, "pass": bool(cond)})
        if not cond:
            print("  FAIL %s" % name)

    # 1) 验收向量：SHA / 严格校验 / 尺寸 / 逐样本组 / CRC
    for ds in vectors["datasets"]:
        path = run / ds["path"]
        data = path.read_bytes()
        chk(ds["name"] + "-sha256", hashlib.sha256(data).hexdigest() == ds["sha256"])
        ok, errs, info = strict_validate(data)
        chk(ds["name"] + "-strict", ok)
        chk(ds["name"] + "-size", len(data) == ds["dma_bytes"])
        h = info["header"]
        chk(ds["name"] + "-fields", (h["width"], h["height"], h["stride_bytes"], h["raw_bytes"],
                                     h["payload_bytes"], h["session_id"], h["capture_seq"]) ==
            (ds["width"], ds["height"], ds["stride_bytes"], ds["raw_bytes"], ds["payload_bytes"],
             h["session_id"], h["capture_seq"]))
        grp = data[HEADER_BYTES:HEADER_BYTES + 5]
        samples = [(grp[k] << 2) | ((grp[4] >> (2 * k)) & 3) for k in range(4)]
        chk(ds["name"] + "-sample-group", samples == ds["pixel_group"])
        chk(ds["name"] + "-raw-crc", info["payload_crc_calc"] == ds["raw_crc32"])
        chk(ds["name"] + "-header-crc", info["header_crc_calc"] == ds["header_crc32"])

        # 2) 全部 frame_mutations 负例（含 valid）
        for v in vectors["frame_mutations"]:
            m = mutate(data, v["operation"])
            okm, _, _ = strict_validate(m)
            chk(ds["name"] + "-" + v["id"], okm == v["expected"])

        # 3) gen 可复现：用向量参数重生成必须逐字节相同
        rx = info["header"]["roi_x"]; ry = info["header"]["roi_y"]
        fl = info["header"]["flags"]
        regen = build_frame(ds["width"], ds["height"], session_id=info["header"]["session_id"],
                            capture_seq=info["header"]["capture_seq"], roi_x=rx, roi_y=ry,
                            bayer_phase=info["header"]["bayer_phase"],
                            camera_id=info["header"]["camera_id"],
                            mode_id=info["header"]["mode_id"], flags=fl)
        chk(ds["name"] + "-regen-byte-identical", regen == data)

    # 4) session 不匹配必须拒
    ds0 = vectors["datasets"][0]
    data0 = (run / ds0["path"]).read_bytes()
    ok_s, _, _ = strict_validate(data0, expect_session=0xDEADBEEF)
    chk("session-mismatch-rejected", not ok_s)
    ok_w, _, _ = strict_validate(data0, expect_width=1920, expect_height=1080)
    chk("geometry-mismatch-rejected", not ok_w)

    bad = sum(1 for r in results if not r["pass"])
    out = {"run_dir": str(run), "checks": len(results), "failures": bad,
           "results": results, "marker": "CONTRACT_VECTORS_OK" if bad == 0 else "CONTRACT_VECTORS_FAIL"}
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"checks": len(results), "failures": bad, "marker": out["marker"]}))
    return 0 if bad == 0 else 1


def main():
    ap = argparse.ArgumentParser(description="R10F v1 非压缩帧严格离线校验/样例生成（T11.rk-align-raw）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("check", help="严格校验一份 R10F 传输内容（.dma / 槽内偏移起 dma_bytes 字节）")
    c.add_argument("file")
    c.add_argument("--session", default=None, help="期望 session（如 0x20261017）；不符即拒")
    c.add_argument("--width", type=int, default=None)
    c.add_argument("--height", type=int, default=None)
    c.add_argument("--json-out", default=None)
    c.set_defaults(func=cmd_check)

    g = sub.add_parser("gen", help="生成合规样例")
    g.add_argument("--preset", choices=list(PRESETS), default="small")
    g.add_argument("--out", required=True)
    g.add_argument("--session", default="0x20261017")
    g.add_argument("--seq", type=int, default=1)
    g.add_argument("--phase", type=int, default=3)
    g.add_argument("--camera", type=int, default=0)
    g.add_argument("--mode", type=int, default=1)
    g.add_argument("--width", type=int, default=None)
    g.add_argument("--height", type=int, default=None)
    g.set_defaults(func=cmd_gen)

    s = sub.add_parser("selftest", help="复算合同验收向量 + 全部负例 + gen 可复现性")
    s.add_argument("--run-dir", default=str(Path(__file__).resolve().parents[2] /
                                            "workflow/runs/20261017-011744-rk-fpga-align"))
    s.add_argument("--json-out", default=None)
    s.set_defaults(func=cmd_selftest)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
