#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R10F 帧查看器：把 FPGA 推来的 R10F 帧变成 PNG，或直接对着设备抓一帧。

两种用法（都只依赖 Python3 标准库，板上不需要 numpy/PIL/cargo）：

  1) 转换已有 dump（例如 rka1_smoke --out 出来的 2592768 字节）：
       python3 r10f_view.py convert /tmp/a.dma -o /tmp/a.png [--scale 2]

  2) 直接从设备抓一帧（ctypes 调 ioctl + mmap，需要在板子上以 root 跑）：
       sudo python3 r10f_view.py snapshot -o /tmp/snap.png [--dev /dev/ANLOGIC-PCI0]
                                              [--timeout 5000] [--last-seq N]

R10F 布局（与 driver/include/anlogic_pci_ioctl.h、r10f_strict.py 一致）：
  槽内偏移 128 起：128B 头 + packed RAW10（stride = width*5/4），末尾 4KiB 对齐补零。
校验只做「够不够解」的轻量检查；严格的合同校验用 tools/pcie_integration/r10f_strict.py。
"""
import argparse
import ctypes
import errno
import fcntl
import mmap
import os
import struct
import sys
import zlib

HDR_BYTES = 128          # 主机侧槽头 + 112B pad
R10F_HDR  = 128

# ---- 与 include/anlogic_pci_ioctl.h 对齐的 ioctl 编码（_IOWR('x', n, struct)）----
IOC_MAGIC = ord('x')

def iowr(nr, size):
    """_IOWR：dir=3（读写）"""
    return (3 << 30) | (size << 16) | (IOC_MAGIC << 8) | nr

def ior(nr, size):
    """_IOR：dir=2（只读）。ANLOGIC_IOCFRAMEINFO 用的是 _IOR，写错方向内核会回 ENOTTY"""
    return (2 << 30) | (size << 16) | (IOC_MAGIC << 8) | nr

class IocFrame(ctypes.Structure):
    _fields_ = [("phys_addr", ctypes.c_ulonglong), ("size", ctypes.c_ulonglong),
                ("stride", ctypes.c_uint), ("slot_bytes", ctypes.c_uint),
                ("hdr_bytes", ctypes.c_uint), ("slots", ctypes.c_uint),
                ("width", ctypes.c_uint), ("height", ctypes.c_uint),
                ("seq", ctypes.c_uint), ("count", ctypes.c_uint),
                ("mb_off", ctypes.c_uint),
                # C 侧 sizeof 会按 8 字节对齐补到 56；ioctl 命令号里带 size，少一个字段就对不上
                ("_pad", ctypes.c_uint)]

class IocFrameWait(ctypes.Structure):
    _fields_ = [("last_seq", ctypes.c_uint), ("timeout_ms", ctypes.c_uint),
                ("seq", ctypes.c_uint), ("count", ctypes.c_uint),
                ("ready", ctypes.c_uint), ("slot", ctypes.c_uint)]

IOCFRAMEINFO = ior(10, ctypes.sizeof(IocFrame))
IOCFRAMEWAIT = iowr(11, ctypes.sizeof(IocFrameWait))

# ---- 方向修正（相机可能被倒装）----
#   none = 原样；h = 左右翻；v = 上下翻；180 = 上下左右都翻（相机旋转 180° 安装时用这个）
def apply_flip(gray, w, h, mode):
    if mode in (None, "", "none"):
        return gray
    rows = [gray[y * w:(y + 1) * w] for y in range(h)]
    if mode in ("h", "180"):
        rows = [r[::-1] for r in rows]
    if mode in ("v", "180"):
        rows = rows[::-1]
    return b"".join(rows)

# ---- 纯标准库 PNG 写出（灰度 8bit）----
def write_png_gray(path, data, w, h):
    raw = bytearray()
    for y in range(h):
        raw.append(0)                       # filter type 0
        raw += data[y * w:(y + 1) * w]
    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)

def parse_header(buf):
    if len(buf) < R10F_HDR:
        raise SystemExit("数据太短，连 128B 头都不够")
    if buf[0:4] != b"R10F":
        raise SystemExit("magic 不是 R10F（前四字节 %r）" % (buf[0:4],))
    w = struct.unpack_from("<H", buf, 0x1C)[0]
    h = struct.unpack_from("<H", buf, 0x1E)[0]
    stride = struct.unpack_from("<I", buf, 0x20)[0]
    pay = struct.unpack_from("<I", buf, 0x30)[0]
    if w == 0 or h == 0 or w % 4 != 0 or stride != w // 4 * 5:
        raise SystemExit("头里的几何不自洽：%dx%d stride=%d" % (w, h, stride))
    if pay < stride * h:
        raise SystemExit("载荷比几何小：payload=%d < stride*h=%d" % (pay, stride * h))
    if len(buf) < R10F_HDR + pay:
        raise SystemExit("载荷不够：需要 %d，实际 %d" % (R10F_HDR + pay, len(buf)))
    return w, h, stride, pay

def unpack_raw10(payload, w, h, stride, scale=1):
    """packed RAW10 → 8bit 灰度（取高 8 位）。scale=2 时做 2x2 平均（Bayer 相位不变的灰度）。"""
    step = scale
    ow, oh = w // step, h // step
    out = bytearray(ow * oh)
    for y in range(oh):
        src_line = (y * step) * stride
        for x in range(ow):
            acc = 0
            for dy in range(step):
                base = src_line + dy * stride
                for dx in range(step):
                    px = x * step + dx
                    g = px // 4
                    p = base + g * 5
                    b0, b1, b2, b3, b4 = payload[p], payload[p + 1], payload[p + 2], payload[p + 3], payload[p + 4]
                    lum = (b0, b1, b2, b3)[px % 4]
                    acc += lum
            out[y * ow + x] = acc // (step * step)
        if (y & 63) == 0:
            print("  解码 %d/%d 行" % (y, oh), file=sys.stderr)
    return bytes(out), ow, oh

def cmd_convert(args):
    data = open(args.file, "rb").read()
    slot = data[HDR_BYTES:] if len(data) > HDR_BYTES and data[0:4] != b"R10F" else data
    w, h, stride, pay = parse_header(slot)
    print("R10F %dx%d stride=%d payload=%d" % (w, h, stride, pay), file=sys.stderr)
    gray, ow, oh = unpack_raw10(slot[R10F_HDR:R10F_HDR + pay], w, h, stride, args.scale)
    gray = apply_flip(gray, ow, oh, args.flip)
    write_png_gray(args.out, gray, ow, oh)
    print("已写出 %s (%dx%d) flip=%s" % (args.out, ow, oh, args.flip))

def cmd_snapshot(args):
    # 只打开 _frame；控制节点不参与抓帧，少开一个 fd 也少一处权限问题。
    frame = os.open(args.dev + "_frame", os.O_RDONLY)
    fi = IocFrame()
    fcntl.ioctl(frame, IOCFRAMEINFO, fi, True)
    if (not fi.width or not fi.height or not fi.stride or
            not fi.slot_bytes or not fi.slots):
        raise SystemExit("驱动报的缓冲几何不完整：%dx%d stride=%d slot_bytes=%d slots=%d"
                         % (fi.width, fi.height, fi.stride, fi.slot_bytes, fi.slots))
    if fi.hdr_bytes != HDR_BYTES:
        raise SystemExit("驱动的 hdr_bytes=%d，本工具按 %d 解析" % (fi.hdr_bytes, HDR_BYTES))
    if fi.stride < fi.hdr_bytes + fi.slot_bytes:
        raise SystemExit("stride=%d 装不下一槽（hdr %d + slot %d）"
                         % (fi.stride, fi.hdr_bytes, fi.slot_bytes))
    if fi.mb_off + 64 > fi.size:
        raise SystemExit("mailbox 偏移 %d + 64 超出缓冲 %d" % (fi.mb_off, fi.size))
    print("缓冲 %dx%d stride=%d slot_bytes=%d slots=%d seq=%d" %
          (fi.width, fi.height, fi.stride, fi.slot_bytes, fi.slots, fi.seq),
          file=sys.stderr)
    last = args.last_seq if args.last_seq >= 0 else fi.seq
    waited = 0
    while waited <= args.timeout:
        fw = IocFrameWait(last_seq=last, timeout_ms=200)
        fcntl.ioctl(frame, IOCFRAMEWAIT, fw, True)
        if fw.ready:
            break
        waited += 200
    else:
        raise SystemExit("等帧超时（%d ms，last_seq=%d）" % (args.timeout, last))
    if fw.slot >= fi.slots:
        raise SystemExit("等帧返回槽 %d，但驱动只报了 %d 槽" % (fw.slot, fi.slots))
    off = fw.slot * fi.stride + fi.hdr_bytes
    if off + fi.slot_bytes > fi.size:
        raise SystemExit("槽 %d 的 [%d,+%d) 超出缓冲 %d"
                         % (fw.slot, off, fi.slot_bytes, fi.size))
    m = mmap.mmap(frame, fi.size, mmap.MAP_SHARED, mmap.PROT_READ)
    slotb = m[off:off + fi.slot_bytes]
    w, h, stride, pay = parse_header(slotb)
    gray, ow, oh = unpack_raw10(slotb[R10F_HDR:R10F_HDR + pay], w, h, stride, args.scale)
    gray = apply_flip(gray, ow, oh, args.flip)
    write_png_gray(args.out, gray, ow, oh)
    if args.raw:
        with open(args.raw, "wb") as f:
            f.write(slotb)
        print("原始槽已写出 %s (%d 字节)" % (args.raw, len(slotb)))
    m.close(); os.close(frame)
    print("已写出 %s (%dx%d)，帧号 %d" % (args.out, ow, oh, fw.seq))

def main():
    ap = argparse.ArgumentParser(description="R10F 帧 → PNG（纯标准库）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("convert", help="转换已有 dump")
    c.add_argument("file")
    c.add_argument("-o", "--out", required=True)
    c.add_argument("--scale", type=int, default=1, choices=(1, 2, 4))
    c.add_argument("--flip", default="none", choices=("none", "h", "v", "180"),
                   help="相机倒装时用 180（上下左右同时翻）")
    c.set_defaults(func=cmd_convert)

    s = sub.add_parser("snapshot", help="从设备直接抓一帧")
    s.add_argument("-o", "--out", required=True)
    s.add_argument("--dev", default="/dev/ANLOGIC-PCI0")
    s.add_argument("--timeout", type=int, default=5000)
    s.add_argument("--last-seq", type=int, default=-1)
    s.add_argument("--raw", default="", help="同时把原始槽字节写到该文件（给 r10f_strict 用）")
    s.add_argument("--scale", type=int, default=1, choices=(1, 2, 4))
    s.add_argument("--flip", default="none", choices=("none", "h", "v", "180"),
                   help="相机倒装时用 180（上下左右同时翻）")
    s.set_defaults(func=cmd_snapshot)

    args = ap.parse_args()
    args.func(args)

if __name__ == "__main__":
    main()
