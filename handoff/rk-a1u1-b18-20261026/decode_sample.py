#!/usr/bin/env python3
"""File-only A1U1 handoff decoder; not a device/DMA receiver."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys
import time
import zlib

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent / "reference"))
import a1_inflate_strict
import a1_ref


def decode_record(data):
    if len(data) < 44:
        raise ValueError("truncated A1U1 header")
    magic, version, flags, hlen, fid, width, height, raw_len, xlen, slen, rc, xc, sc, hc = struct.unpack_from("<4sBBHIHHIIIIIII", data)
    if magic != b"A1U1" or version != 1 or hlen != 44:
        raise ValueError("unsupported A1U1 header")
    if flags & ~3:
        raise ValueError("unknown transform flags")
    prep, pred = flags & 1, (flags >> 1) & 1
    a1_ref.validate_geometry(width, height, prep, pred, raw_len)
    if raw_len > 64 * 1024 * 1024 or xlen != raw_len:
        raise ValueError("invalid decoded length")
    if not 0 < slen <= raw_len + 4096 or len(data) != hlen + slen:
        raise ValueError("record length mismatch or trailing bytes")
    if zlib.crc32(data[:40]) & 0xFFFFFFFF != hc:
        raise ValueError("header CRC mismatch")
    stream = data[hlen:]
    if zlib.crc32(stream) & 0xFFFFFFFF != sc:
        raise ValueError("stream CRC mismatch")
    xform, stats = a1_inflate_strict.inflate(stream, expected_len=xlen)
    if zlib.crc32(xform) & 0xFFFFFFFF != xc:
        raise ValueError("transform CRC mismatch")
    raw = a1_ref.inverse_frame(xform, width, height, prep, pred)
    if len(raw) != raw_len or zlib.crc32(raw) & 0xFFFFFFFF != rc:
        raise ValueError("RAW length or CRC mismatch")
    result = dict(status="pass", frame_id=fid, version=version, flags=flags,
                  width=width, height=height, stride=width * 5 // 4,
                  raw_len=raw_len, stream_len=slen,
                  header_crc32=f"{hc:08x}", stream_crc32=f"{sc:08x}",
                  xform_crc32=f"{xc:08x}", raw_crc32=f"{rc:08x}",
                  input_sha256=hashlib.sha256(data).hexdigest(),
                  raw_sha256=hashlib.sha256(raw).hexdigest(), strict_stats=stats)
    return raw, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    started = time.perf_counter()
    try:
        raw, result = decode_record(args.input.read_bytes())
    except (ValueError, a1_inflate_strict.A1Error, OSError) as error:
        print(json.dumps(dict(status="fail", error=str(error))))
        return 1
    result["file_decode_seconds"] = time.perf_counter() - started
    args.out_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.out_dir / "frame.raw10"
    result_path = args.out_dir / "result.json"
    if raw_path.exists() or result_path.exists():
        raise FileExistsError("output already exists; choose a new --out-dir")
    raw_path.write_bytes(raw)
    result_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
