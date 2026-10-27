#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T10 A+ v1 independent strict DEFLATE/zlib parser (P1, test-only).

Subset: zlib header 0x78 0x9C, stored (BTYPE=00), fixed Huffman (BTYPE=01),
Adler-32. Dynamic Huffman is intentionally rejected. This module does not
import a1_ref.py and does not call zlib; it is the second, independent decoder
used by tools/codec/a1_p1_verify.py.

Strictness:
  * exactly one zlib member, no trailing bytes;
  * header FCHECK/FDICT checks;
  * stored LEN/NLEN and LEN<=4800;
  * block-header and final padding bits must be zero;
  * distance 1..2048 and never beyond current output;
  * fixed symbols 286/287 and distance symbols 30/31 rejected;
  * decoded length must equal the expected xform_len when supplied;
  * output hard limit 64 MiB.

Reason codes are stable strings (see format-v1.md §7).
"""
import struct
import sys

sys.dont_write_bytecode = True

E_TRUNCATED = "E_TRUNCATED"
E_BAD_ZLIB_HEADER = "E_BAD_ZLIB_HEADER"
E_FDICT_SET = "E_FDICT_SET"
E_BAD_BTYPE_DYNAMIC = "E_BAD_BTYPE_DYNAMIC"
E_BAD_BTYPE_RESERVED = "E_BAD_BTYPE_RESERVED"
E_BAD_STORED_NLEN = "E_BAD_STORED_NLEN"
E_BAD_STORED_LEN = "E_BAD_STORED_LEN"
E_BAD_PADDING = "E_BAD_PADDING"
E_INVALID_HUFFMAN_CODE = "E_INVALID_HUFFMAN_CODE"
E_BAD_LENGTH_CODE = "E_BAD_LENGTH_CODE"
E_BAD_DISTANCE_CODE = "E_BAD_DISTANCE_CODE"
E_BAD_DISTANCE = "E_BAD_DISTANCE"
E_DISTANCE_WINDOW = "E_DISTANCE_WINDOW"
E_BAD_ADLER = "E_BAD_ADLER"
E_LENGTH_MISMATCH = "E_LENGTH_MISMATCH"
E_TRAILING_BYTES = "E_TRAILING_BYTES"
E_OUTPUT_LIMIT = "E_OUTPUT_LIMIT"

MAX_DIST = 2048
MAX_BLOCK_BYTES = 4800
MAX_OUTPUT = 64 * 1024 * 1024


class A1Error(Exception):
    def __init__(self, code, detail=""):
        self.code = code
        self.detail = detail
        super().__init__(code + (": " + detail if detail else ""))


def adler32(data):
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


class BitReader:
    __slots__ = ("data", "pos", "nbits")

    def __init__(self, data):
        self.data = bytes(data)
        self.pos = 0
        self.nbits = len(self.data) * 8

    def _fetch(self, p, w):
        byte0 = p >> 3
        off = p & 7
        nbytes = (off + w + 7) // 8
        chunk = int.from_bytes(self.data[byte0:byte0 + nbytes], "little")
        return (chunk >> off) & ((1 << w) - 1)

    def bits(self, n):
        if n <= 0:
            return 0
        if self.pos + n > self.nbits:
            raise A1Error(E_TRUNCATED, "need %d bits at %d of %d" % (n, self.pos, self.nbits))
        v = self._fetch(self.pos, n)
        self.pos += n
        return v

    def peek(self, n):
        avail = self.nbits - self.pos
        if avail <= 0:
            raise A1Error(E_TRUNCATED, "peek at end")
        use = n if n < avail else avail
        v = self._fetch(self.pos, use)
        return v, use

    def align_zero(self, code=E_BAD_PADDING):
        pad = (-self.pos) & 7
        if pad:
            if self.bits(pad) != 0:
                raise A1Error(code, "non-zero padding at bit %d" % self.pos)
        return pad

    def read_bytes(self, n):
        if self.pos & 7:
            raise AssertionError("read_bytes requires alignment")
        if self.pos + 8 * n > self.nbits:
            raise A1Error(E_TRUNCATED, "need %d bytes at %d of %d" % (n, self.pos >> 3, len(self.data)))
        b = self.pos >> 3
        self.pos += 8 * n
        return self.data[b:b + n]


LEN_BASE = (3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
            35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258)
LEN_EXTRA = (0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
             3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0)
DIST_BASE = (1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193,
             257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145,
             8193, 12289, 16385, 24577)
DIST_EXTRA = (0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6,
              7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13)


def _build_table(lengths):
    max_bits = max(lengths) if lengths else 0
    bl_count = [0] * (max_bits + 1)
    for ln in lengths:
        if ln:
            bl_count[ln] += 1
    code = 0
    next_code = [0] * (max_bits + 1)
    for bits in range(1, max_bits + 1):
        code = (code + bl_count[bits - 1]) << 1
        next_code[bits] = code
    table = {}
    for sym, ln in enumerate(lengths):
        if ln:
            table[(ln, next_code[ln])] = sym
            next_code[ln] += 1
    return table, max_bits


def _fixed_tables():
    lit_len = [8] * 144 + [9] * 112 + [7] * 24 + [8] * 8
    dist_len = [5] * 32
    lit_tab, lit_max = _build_table(lit_len)
    dist_tab, dist_max = _build_table(dist_len)
    return lit_tab, lit_max, dist_tab, dist_max


_LIT_TAB, _LIT_MAX, _DIST_TAB, _DIST_MAX = _fixed_tables()


_REV_CACHE = {}


def _rev_bits(v, n):
    """Reverse n LSB-first stream bits into an MSB-first Huffman code."""
    key = (v, n)
    r = _REV_CACHE.get(key)
    if r is None:
        r = 0
        x = v
        for _ in range(n):
            r = (r << 1) | (x & 1)
            x >>= 1
        _REV_CACHE[key] = r
    return r


def _decode_symbol(br, tab, max_bits):
    val, avail = br.peek(max_bits)
    val = _rev_bits(val, avail)
    if avail < max_bits:
        val <<= (max_bits - avail)
    lim = max_bits if max_bits < avail else avail
    for ln in range(1, lim + 1):
        sym = tab.get((ln, val >> (max_bits - ln)))
        if sym is not None:
            br.pos += ln
            return sym
    if avail < max_bits:
        raise A1Error(E_TRUNCATED, "huffman code cut at %d" % br.pos)
    raise A1Error(E_INVALID_HUFFMAN_CODE, "no fixed code at %d" % br.pos)


def _stored_block(br, out):
    br.align_zero()
    ln = br.bits(16)
    nlen = br.bits(16)
    if (ln ^ 0xFFFF) != nlen:
        raise A1Error(E_BAD_STORED_NLEN, "LEN=%d NLEN=%d" % (ln, nlen))
    if ln > MAX_BLOCK_BYTES:
        raise A1Error(E_BAD_STORED_LEN, "LEN=%d" % ln)
    out += br.read_bytes(ln)
    return ln


def _fixed_block(br, out):
    ntok = 0
    nlit = 0
    nmatch = 0
    while True:
        sym = _decode_symbol(br, _LIT_TAB, _LIT_MAX)
        if sym < 256:
            out.append(sym)
            nlit += 1
            ntok += 1
            if len(out) > MAX_OUTPUT:
                raise A1Error(E_OUTPUT_LIMIT)
            continue
        if sym == 256:
            break
        if sym > 285:
            raise A1Error(E_BAD_LENGTH_CODE, "sym=%d" % sym)
        idx = sym - 257
        length = LEN_BASE[idx]
        eb = LEN_EXTRA[idx]
        if eb:
            length += br.bits(eb)
        dsym = _decode_symbol(br, _DIST_TAB, _DIST_MAX)
        if dsym > 29:
            raise A1Error(E_BAD_DISTANCE_CODE, "dist_sym=%d" % dsym)
        dist = DIST_BASE[dsym]
        deb = DIST_EXTRA[dsym]
        if deb:
            dist += br.bits(deb)
        if dist > MAX_DIST:
            raise A1Error(E_DISTANCE_WINDOW, "dist=%d" % dist)
        if dist > len(out):
            raise A1Error(E_BAD_DISTANCE, "dist=%d out=%d" % (dist, len(out)))
        for _ in range(length):
            out.append(out[-dist])
        ntok += 1
        nmatch += 1
        if len(out) > MAX_OUTPUT:
            raise A1Error(E_OUTPUT_LIMIT)
    return {"tokens": ntok, "literals": nlit, "matches": nmatch}


def inflate(data, expected_len=None):
    """Parse one zlib member. Returns (out_bytes, stats). Raises A1Error."""
    br = BitReader(data)
    cmf = br.bits(8)
    flg = br.bits(8)
    if (cmf & 0x0F) != 8:
        raise A1Error(E_BAD_ZLIB_HEADER, "CM=%d" % (cmf & 0x0F))
    if (cmf >> 4) > 7:
        raise A1Error(E_BAD_ZLIB_HEADER, "CINFO=%d" % (cmf >> 4))
    if ((cmf << 8) | flg) % 31 != 0:
        raise A1Error(E_BAD_ZLIB_HEADER, "FCHECK")
    if flg & 0x20:
        raise A1Error(E_FDICT_SET, "FDICT=1")

    out = bytearray()
    stats = {"blocks": 0, "stored_blocks": 0, "fixed_blocks": 0,
             "literals": 0, "matches": 0, "stored_bytes": 0}
    while True:
        stats["blocks"] += 1
        if stats["blocks"] > 100000:
            raise A1Error(E_OUTPUT_LIMIT, "too many blocks")
        bfinal = br.bits(1)
        btype = br.bits(2)
        if btype == 0:
            stats["stored_blocks"] += 1
            stats["stored_bytes"] += _stored_block(br, out)
        elif btype == 1:
            stats["fixed_blocks"] += 1
            r = _fixed_block(br, out)
            stats["literals"] += r["literals"]
            stats["matches"] += r["matches"]
        elif btype == 2:
            raise A1Error(E_BAD_BTYPE_DYNAMIC, "dynamic Huffman not in v1 subset")
        else:
            raise A1Error(E_BAD_BTYPE_RESERVED, "BTYPE=3")
        if bfinal:
            break

    br.align_zero()
    adler = int.from_bytes(br.read_bytes(4), "big")
    if br.pos != br.nbits:
        raise A1Error(E_TRAILING_BYTES, "pos=%d nbits=%d" % (br.pos, br.nbits))
    calc = adler32(out)
    if adler != calc:
        raise A1Error(E_BAD_ADLER, "stored=%08X calc=%08X" % (adler, calc))
    if expected_len is not None and len(out) != expected_len:
        raise A1Error(E_LENGTH_MISMATCH, "got=%d expected=%d" % (len(out), expected_len))
    stats["out_len"] = len(out)
    return bytes(out), stats


def main(argv):
    import argparse
    ap = argparse.ArgumentParser(description="strict A+ v1 zlib/DEFLATE parser")
    ap.add_argument("stream")
    ap.add_argument("--out")
    ap.add_argument("--expected-len", type=int, default=None)
    args = ap.parse_args(argv)
    data = open(args.stream, "rb").read()
    try:
        out, st = inflate(data, args.expected_len)
    except A1Error as e:
        print("STRICT_REJECT %s %s" % (e.code, e.detail))
        return 1
    if args.out:
        open(args.out, "wb").write(out)
    print("STRICT_OK out_len=%d blocks=%d stored=%d fixed=%d literals=%d matches=%d sha256=%s"
          % (len(out), st["blocks"], st["stored_blocks"], st["fixed_blocks"],
             st["literals"], st["matches"],
             __import__("hashlib").sha256(out).hexdigest()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
