#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T10 A+ v1 reference encoder model (P1, test-only).

Implements the frozen format in
workflow/runs/20261020-0209_singleA_aplus_closedloop/T10.a1-format-ref/p1/format-v1.md:

  packed RAW10 (5B/4px) -> declared RGGB phase-0 Bayer plane reorder
  (+ optional per-plane modulo-1024 prediction) -> real LZ77
  -> fixed-Huffman candidate / block-level stored fallback
  -> zlib wrapper (0x78 0x9C + Adler-32).

This module is an ENCODER MODEL plus the transform. It deliberately does not
decode or parse DEFLATE; that is done by the independent strict parser and by
Python zlib in the verifier. Do not use this module as a production codec.

No file/board/production state is touched by this module.
"""
import binascii
import hashlib
import struct
import sys

sys.dont_write_bytecode = True

TEST_MAGIC = b"A1T1"
TEST_VERSION = 1
TEST_HEADER_BYTES = 32
ZLIB_CMF = 0x78
ZLIB_FLG = 0x9C
RAW10_BYTES_PER_GROUP = 5
RAW10_PIXELS_PER_GROUP = 4
MIN_MATCH = 3
MAX_MATCH = 258
MAX_DIST = 2048
HASH_BITS = 12
HASH_SIZE = 1 << HASH_BITS
MAX_BLOCK_BYTES = 4800


def crc32(data):
    return binascii.crc32(bytes(data)) & 0xFFFFFFFF


def adler32(data):
    """RFC1950 Adler-32, independent of zlib."""
    a = 1
    b = 0
    mod = 65521
    for i in range(0, len(data), 5552):
        for x in data[i:i + 5552]:
            a += x
            b += a
        a %= mod
        b %= mod
    return ((b << 16) | a) & 0xFFFFFFFF


def sha256(data):
    return hashlib.sha256(bytes(data)).hexdigest()


# ---------------------------------------------------------------------------
# packed RAW10 <-> samples
# ---------------------------------------------------------------------------
def unpack_groups(buf, ngroups=None):
    buf = bytes(buf)
    if ngroups is None:
        ngroups = len(buf) // RAW10_BYTES_PER_GROUP
    if len(buf) != ngroups * RAW10_BYTES_PER_GROUP:
        raise ValueError("packed RAW10 buffer length is not a multiple of 5")
    out = []
    ap = out.append
    for i in range(ngroups):
        o = i * 5
        b0 = buf[o]
        b1 = buf[o + 1]
        b2 = buf[o + 2]
        b3 = buf[o + 3]
        b4 = buf[o + 4]
        ap((b0 << 2) | (b4 & 0x3))
        ap((b1 << 2) | ((b4 >> 2) & 0x3))
        ap((b2 << 2) | ((b4 >> 4) & 0x3))
        ap((b3 << 2) | ((b4 >> 6) & 0x3))
    return out


def pack_samples(samples):
    n = len(samples)
    if n % RAW10_PIXELS_PER_GROUP:
        raise ValueError("sample count is not a multiple of 4")
    out = bytearray()
    ap = out.append
    for i in range(0, n, 4):
        p0 = samples[i]
        p1 = samples[i + 1]
        p2 = samples[i + 2]
        p3 = samples[i + 3]
        if (p0 | p1 | p2 | p3) > 0x3FF:
            raise ValueError("sample exceeds 10 bits")
        ap(p0 >> 2)
        ap(p1 >> 2)
        ap(p2 >> 2)
        ap(p3 >> 2)
        ap(((p3 & 3) << 6) | ((p2 & 3) << 4) | ((p1 & 3) << 2) | (p0 & 3))
    return bytes(out)


# ---------------------------------------------------------------------------
# declared RGGB phase-0 Bayer transform
# ---------------------------------------------------------------------------
def validate_geometry(w, h, prep_en, pred_en, raw_len=None):
    if w < 4 or w % 4 != 0:
        raise ValueError("W must be >=4 and W%4==0")
    if h < 2:
        raise ValueError("H must be >=2")
    if pred_en and not prep_en:
        raise ValueError("pred_en=1 requires prep_en=1")
    stride = w * RAW10_BYTES_PER_GROUP // RAW10_PIXELS_PER_GROUP
    if raw_len is not None and raw_len != stride * h:
        raise ValueError("raw_len does not equal stride*H")
    return stride


def bayer_forward_block(raw_block, w, nrows, pred_en):
    """Declared RGGB block transform: 2 rows (or 1 tail row) -> xform bytes."""
    stride = w * 5 // 4
    if nrows not in (1, 2):
        raise ValueError("nrows must be 1 or 2")
    if len(raw_block) != stride * nrows:
        raise ValueError("block length mismatch")
    rows = []
    for r in range(nrows):
        seg = raw_block[r * stride:(r + 1) * stride]
        rows.append(unpack_groups(seg, w // 4))

    planes = [rows[0][0::2], rows[0][1::2]]
    if nrows == 2:
        planes.append(rows[1][0::2])
        planes.append(rows[1][1::2])

    seq = []
    if pred_en:
        for p in planes:
            prev = 0
            for s in p:
                seq.append((s - prev) & 0x3FF)
                prev = s
    else:
        for p in planes:
            seq.extend(p)
    return pack_samples(seq)


def bayer_inverse_block(xform_block, w, nrows, pred_en):
    """Inverse of bayer_forward_block; returns packed RAW10 bytes."""
    stride = w * 5 // 4
    if nrows not in (1, 2):
        raise ValueError("nrows must be 1 or 2")
    if len(xform_block) != stride * nrows:
        raise ValueError("xform block length mismatch")
    seq = unpack_groups(xform_block, len(xform_block) // 5)
    nplanes = 4 if nrows == 2 else 2
    plen = w // 2
    if len(seq) != nplanes * plen:
        raise ValueError("xform sample count mismatch")
    planes = []
    for k in range(nplanes):
        p = seq[k * plen:(k + 1) * plen]
        if pred_en:
            acc = 0
            rec = []
            for d in p:
                acc = (acc + d) & 0x3FF
                rec.append(acc)
            p = rec
        planes.append(p)

    rows = [[0] * w for _ in range(nrows)]
    rows[0][0::2] = planes[0]
    rows[0][1::2] = planes[1]
    if nrows == 2:
        rows[1][0::2] = planes[2]
        rows[1][1::2] = planes[3]
    out = bytearray()
    for row in rows:
        out += pack_samples(row)
    return bytes(out)


def transform_frame(raw, w, h, prep_en, pred_en):
    stride = validate_geometry(w, h, prep_en, pred_en, len(raw))
    if not prep_en:
        return bytes(raw)
    out = bytearray()
    y = 0
    while y < h:
        nrows = 2 if (y + 1) < h else 1
        out += bayer_forward_block(raw[y * stride:(y + nrows) * stride], w, nrows, pred_en)
        y += nrows
    if len(out) != len(raw):
        raise AssertionError("transform length changed")
    return bytes(out)


def inverse_frame(xform, w, h, prep_en, pred_en):
    stride = validate_geometry(w, h, prep_en, pred_en, len(xform))
    if not prep_en:
        return bytes(xform)
    out = bytearray()
    y = 0
    while y < h:
        nrows = 2 if (y + 1) < h else 1
        out += bayer_inverse_block(xform[y * stride:(y + nrows) * stride], w, nrows, pred_en)
        y += nrows
    if len(out) != len(xform):
        raise AssertionError("inverse length changed")
    return bytes(out)


def split_blocks(xform, w, h):
    stride = w * 5 // 4
    blocks = []
    y = 0
    while y < h:
        nrows = 2 if (y + 1) < h else 1
        blocks.append(xform[y * stride:(y + nrows) * stride])
        y += nrows
    if sum(len(b) for b in blocks) != len(xform):
        raise AssertionError("block split lost bytes")
    return blocks


# ---------------------------------------------------------------------------
# bit writer (DEFLATE is LSB-first; Huffman codes are written MSB-first)
# ---------------------------------------------------------------------------
class BitWriter:
    __slots__ = ("acc", "n", "out")

    def __init__(self):
        self.acc = 0
        self.n = 0
        self.out = bytearray()

    def bits(self, value, nbits):
        if nbits <= 0:
            return
        self.acc |= (value & ((1 << nbits) - 1)) << self.n
        self.n += nbits
        while self.n >= 8:
            self.out.append(self.acc & 0xFF)
            self.acc >>= 8
            self.n -= 8

    def align(self):
        if self.n:
            self.out.append(self.acc & 0xFF)
            self.acc = 0
            self.n = 0

    def write_bytes(self, data):
        if self.n:
            raise AssertionError("write_bytes requires byte alignment")
        self.out += data


def _rev(code, n):
    r = 0
    for _ in range(n):
        r = (r << 1) | (code & 1)
        code >>= 1
    return r


# RFC1951 fixed Huffman: (reversed code, bit length) for LSB-first output.
FIXED_LIT = [None] * 288
for _i in range(0, 144):
    FIXED_LIT[_i] = (_rev(0x30 + _i, 8), 8)
for _i in range(144, 256):
    FIXED_LIT[_i] = (_rev(0x190 + (_i - 144), 9), 9)
for _i in range(256, 280):
    FIXED_LIT[_i] = (_rev(_i - 256, 7), 7)
for _i in range(280, 288):
    FIXED_LIT[_i] = (_rev(0xC0 + (_i - 280), 8), 8)
FIXED_DIST = [(_rev(_i, 5), 5) for _i in range(32)]

LEN_BASE = (3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
            35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258)
LEN_EXTRA = (0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
             3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0)
DIST_BASE = (1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193,
             257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145,
             8193, 12289, 16385, 24577)
DIST_EXTRA = (0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6,
              7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13)

LEN_MAP = {}
for _k, (_base, _eb) in enumerate(zip(LEN_BASE, LEN_EXTRA)):
    _sym = 257 + _k
    _span = (1 << _eb) if _eb else 1
    for _l in range(_base, _base + _span):
        LEN_MAP[_l] = (_sym, _l - _base, _eb)
LEN_MAP[258] = (285, 0, 0)  # canonical shortest encoding of the max length

DIST_MAP = {}
for _k, (_base, _eb) in enumerate(zip(DIST_BASE, DIST_EXTRA)):
    _span = (1 << _eb) if _eb else 1
    for _d in range(_base, _base + _span):
        DIST_MAP[_d] = (_k, _d - _base, _eb)


# ---------------------------------------------------------------------------
# LZ77 + fixed-Huffman candidate + block stored fallback
# ---------------------------------------------------------------------------
def _tokenize(data):
    """Greedy LZ77 with a 4096-entry 3-byte hash and one candidate.

    History is block-local: distance never exceeds the current position, and
    the table is fresh for every block (the caller calls this per block).
    """
    n = len(data)
    table = [-1] * HASH_SIZE
    toks = []
    i = 0
    while i < n:
        best_len = 0
        best_dist = 0
        if i + 2 < n:
            h = ((data[i] * 251 + data[i + 1]) * 251 + data[i + 2]) & (HASH_SIZE - 1)
            cand = table[h]
            if cand >= 0:
                dist = i - cand
                if dist <= MAX_DIST:
                    maxlen = min(MAX_MATCH, n - i)
                    l = 0
                    while l < maxlen and data[cand + l] == data[i + l]:
                        l += 1
                    if l >= MIN_MATCH:
                        best_len = l
                        best_dist = dist
            table[h] = i
        if best_len >= MIN_MATCH:
            toks.append((1, best_len, best_dist))
            end = i + best_len
            j = i + 1
            while j < end:
                if j + 2 < n:
                    h = ((data[j] * 251 + data[j + 1]) * 251 + data[j + 2]) & (HASH_SIZE - 1)
                    table[h] = j
                j += 1
            i = end
        else:
            toks.append((0, data[i], 0))
            i += 1
    return toks


def _fixed_cost(toks):
    bits = 3 + 7  # BFINAL/BTYPE + EOB(256)
    for t in toks:
        if t[0] == 0:
            bits += FIXED_LIT[t[1]][1]
        else:
            ls, _lev, leb = LEN_MAP[t[1]]
            ds, _dev, deb = DIST_MAP[t[2]]
            bits += FIXED_LIT[ls][1] + leb + FIXED_DIST[ds][1] + deb
    return bits


def _stored_cost(bit_offset, nbytes):
    pad = (8 - ((bit_offset + 3) % 8)) % 8
    return 3 + pad + 32 + 8 * nbytes


def _write_fixed(w, toks, final):
    w.bits(1 if final else 0, 1)
    w.bits(1, 2)  # BTYPE=01
    for t in toks:
        if t[0] == 0:
            code, nbits = FIXED_LIT[t[1]]
            w.bits(code, nbits)
        else:
            ls, lev, leb = LEN_MAP[t[1]]
            code, nbits = FIXED_LIT[ls]
            w.bits(code, nbits)
            if leb:
                w.bits(lev, leb)
            ds, dev, deb = DIST_MAP[t[2]]
            code, nbits = FIXED_DIST[ds]
            w.bits(code, nbits)
            if deb:
                w.bits(dev, deb)
    code, nbits = FIXED_LIT[256]
    w.bits(code, nbits)


def _write_stored(w, data, final):
    if len(data) > 0xFFFF:
        raise ValueError("stored block too large")
    w.bits(1 if final else 0, 1)
    w.bits(0, 2)  # BTYPE=00
    w.align()
    ln = len(data)
    w.write_bytes(struct.pack("<HH", ln, (~ln) & 0xFFFF))
    w.write_bytes(data)


def deflate_blocks(blocks):
    """Return (deflate_body_bytes, stats). Adler/header are added by caller."""
    if not blocks:
        raise ValueError("no blocks")
    w = BitWriter()
    stats = {
        "block_count": len(blocks),
        "fixed_blocks": 0,
        "stored_blocks": 0,
        "literal_count": 0,
        "match_count": 0,
        "max_match_len": 0,
        "max_match_dist": 0,
        "block_types": [],
        "block_sizes": [],
        "fixed_bits": [],
        "stored_bits": [],
    }
    for idx, b in enumerate(blocks):
        if not b:
            raise ValueError("empty block")
        if len(b) > MAX_BLOCK_BYTES:
            raise ValueError("block exceeds 4800 B")
        final = idx == (len(blocks) - 1)
        toks = _tokenize(b)
        fixed_bits = _fixed_cost(toks)
        stored_bits = _stored_cost(w.n, len(b))
        use_stored = fixed_bits >= stored_bits
        if use_stored:
            _write_stored(w, b, final)
            stats["stored_blocks"] += 1
            stats["block_types"].append("stored")
        else:
            _write_fixed(w, toks, final)
            stats["fixed_blocks"] += 1
            stats["block_types"].append("fixed")
        stats["block_sizes"].append(len(b))
        stats["fixed_bits"].append(fixed_bits)
        stats["stored_bits"].append(stored_bits)
        for t in toks:
            if t[0] == 0:
                stats["literal_count"] += 1
            else:
                stats["match_count"] += 1
                if t[1] > stats["max_match_len"]:
                    stats["max_match_len"] = t[1]
                if t[2] > stats["max_match_dist"]:
                    stats["max_match_dist"] = t[2]
    stats["tail_bit_offset"] = w.n
    stats["tail_pad_bits"] = (8 - w.n) % 8
    w.align()
    return bytes(w.out), stats


def build_record(raw, xform, stream, prep_en, pred_en):
    flags = (1 if prep_en else 0) | (2 if pred_en else 0)
    hdr = bytearray(TEST_HEADER_BYTES)
    struct.pack_into("<4sHHIIIIII", hdr, 0,
                     TEST_MAGIC, TEST_VERSION, flags,
                     len(raw), len(xform), len(stream),
                     crc32(raw), crc32(xform), 0)
    # header CRC covers [0x00, 0x1C) only; the field itself is not input.
    struct.pack_into("<I", hdr, 0x1C, crc32(bytes(hdr[:0x1C])))
    return bytes(hdr) + bytes(stream)


def fix_record_crc(rec):
    b = bytearray(rec)
    if len(b) < TEST_HEADER_BYTES:
        raise ValueError("record too short")
    b[0x1C:0x20] = b"\x00\x00\x00\x00"
    struct.pack_into("<I", b, 0x1C, crc32(bytes(b[:0x1C])))
    return bytes(b)


def encode_frame(raw, w, h, prep_en, pred_en):
    raw = bytes(raw)
    stride = validate_geometry(w, h, prep_en, pred_en, len(raw))
    xform = transform_frame(raw, w, h, prep_en, pred_en)
    blocks = split_blocks(xform, w, h)
    body, stats = deflate_blocks(blocks)
    adler = adler32(xform)
    stream = bytes([ZLIB_CMF, ZLIB_FLG]) + body + adler.to_bytes(4, "big")
    rec = build_record(raw, xform, stream, prep_en, pred_en)
    out = {
        "raw": raw,
        "xform": xform,
        "stream": stream,
        "record": rec,
        "w": w,
        "h": h,
        "prep_en": prep_en,
        "pred_en": pred_en,
        "stride": stride,
        "raw_len": len(raw),
        "xform_len": len(xform),
        "stream_len": len(stream),
        "raw_crc32": crc32(raw),
        "xform_crc32": crc32(xform),
        "stream_crc32": crc32(stream),
        "zlib_adler32": adler,
        "ratio_raw_over_stream": (len(raw) / len(stream)) if stream else 0.0,
        "expansion_pct": (len(stream) / len(raw) - 1.0) * 100.0 if raw else 0.0,
        "stats": stats,
    }
    out["sha256"] = {
        "raw": sha256(raw),
        "xform": sha256(xform),
        "stream": sha256(stream),
        "record": sha256(rec),
    }
    return out
