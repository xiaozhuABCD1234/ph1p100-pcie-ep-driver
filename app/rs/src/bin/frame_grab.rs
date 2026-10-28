// SPDX-License-Identifier: GPL-2.0
/*!
 * frame_grab —— 从驱动分配的收帧缓冲（DMA）里取一帧，写成 PGM
 * （原 app/frame_grab.c 的 Rust 版，行为不变）
 *
 * 这是「收帧缓冲」这一层的最小用户态：只做三件事 ——
 *   1) ANLOGIC_IOCFRAMEINFO 问出缓冲的物理地址、大小、几何与序号/计数；
 *   2) ANLOGIC_IOCFRAMEWAIT 等一帧就绪（轮询序号，本 EP 没有 MSI，不走中断）；
 *   3) 把那一槽像素写成 P5 灰度 PGM。
 *
 * 槽内容若以 R10F 开头（128B 头 + packed RAW10 + 4KiB 对齐零尾），先走 r10f::parse
 * 严格校验，再把 packed RAW10 解成 8bit 灰度预览（每采样取高 8 位）写 PGM；旧的
 * 8bit 槽内容仍按原样直拷，行为不变。
 *
 * 缓冲布局与 FPGA 的握手约定（握手时序、槽头字段）写在 include/anlogic_pci_ioctl.h：
 * 槽区从缓冲起点开始，槽 n 在 phys_addr + stride*n（槽头后才是像素），mailbox
 * 描述符在槽区之后、由 ioctl 报的 mb_off 定位（整页，不与槽区重叠）。本工具只读：
 *   - 打开设备用只读；mmap 用 PROT_READ（驱动也会拒绝可写映射）；
 *   - 不碰任何 BAR 寄存器，所以寄存器访问规约在这一层没有适用对象
 *     （该规约管的是对 EP 的 MMIO，本层读写的是系统内存）。
 *
 * 空转行为（板上还没接 FPGA 帧源时必须正常返回）：mailbox.seq 恒 0，
 * ANLOGIC_IOCFRAMEWAIT 到时返回 ready=0，本工具打印「无新帧」并以退出码 4 结束，
 * 不写文件、不报错。
 *
 * 退出码
 *   0 成功（PGM 已写出）
 *   1 命令行用法错误
 *   2 设备不可用（open/flock/ioctl/mmap 失败、缓冲描述符不符）
 *   3 收帧缓冲还没就绪（描述符与驱动报的不一致）
 *   4 等超时 / 没有新帧（空转的正常结果）
 *   5 帧一致性不过（槽头序号对不上、帧号在搬运期间绕回、长度不合理）
 *   6 输出文件或内存错误
 *
 * 用法：frame_grab [-d /dev/ANLOGIC-PCI0_frame] [-t 超时ms] [-n] [-o 输出.pgm] [--info]
 *       frame_grab --pair [--ctrl /dev/ANLOGIC-PCI0_control] -o A.pgm --out-b B.pgm
 *       -o        输出 PGM 文件名（默认 dma_frame.pgm），也可以直接写成位置参数
 *       -n        只在出现比当前更新的帧时返回（默认：缓冲里已有帧就直接取最新一帧）
 *       --info    只打印缓冲的物理地址与几何，不取帧（空转自检用）
 */
use std::borrow::Cow;
use std::ffi::c_void;
use std::io::Write;
use std::os::fd::{AsRawFd, RawFd};

use anlogic_pci_tools::getopt::{Getopt, LongOpt};
use anlogic_pci_tools::ioctl::{
    FrameMailbox, FrameSlotHdr, IocBar, IocFrame, IocFrameWait, ANLOGIC_FRAME_FLAG_COMPLETE,
    ANLOGIC_FRAME_MAGIC, ANLOGIC_FRAME_MAX_WAIT_MS, ANLOGIC_IOCFRAMEINFO, ANLOGIC_IOCFRAMEWAIT,
    ANLOGIC_IOCR, ANLOGIC_IOCW,
};
use anlogic_pci_tools::parse::strtoul_base0;
use anlogic_pci_tools::rka1::r10f;
use anlogic_pci_tools::sys::{self, Mmap, PROT_READ};

const DEF_DEV: &str = "/dev/ANLOGIC-PCI0_frame";
const DEF_CTRL: &str = "/dev/ANLOGIC-PCI0_control";
const DEF_OUT: &str = "dma_frame.pgm";
const DEF_OUT_B: &str = "dma_frame_b.pgm";
const DEF_TIMEOUT_MS: u32 = 3000;

/// BAR0 双路推帧寄存器（与驱动头文件、ep_bar_win.v 同步）
const BAR_CTRL: u32 = 0x44;
const BAR_TX_CAP: u32 = 0x100;
const BAR_TX_SESSION: u32 = 0x104;
const BAR_TX_CMD: u32 = 0x108;
const BAR_TX_ACK_SEQ: u32 = 0x10C;
const BAR_PUSH_DST_LO: u32 = 0xB0;
const BAR_PUSH_DST_HI: u32 = 0xB4;
const BAR_PUSH_DB_LO: u32 = 0xB8;
const BAR_PUSH_DB_HI: u32 = 0xBC;
const BAR_PUSH_DONE: u32 = 0xC0;
const BAR_PUSH_STATUS: u32 = 0xC4;
const BAR_PUSH_ERR: u32 = 0xC8;
const BAR_PAIR_ACK_SEQ: u32 = 0x220;
const BAR_PAIR_DST_LO: u32 = 0x224;
const BAR_PAIR_DST_HI: u32 = 0x228;
const BAR_PAIR_DB_LO: u32 = 0x22C;
const BAR_PAIR_DB_HI: u32 = 0x230;
const BAR_PAIR_DONE: u32 = 0x210;
const BAR_PAIR_STATUS: u32 = 0x214;
const BAR_PAIR_ERR: u32 = 0x218;
const CTRL_PAIR_BITS: u32 = 0x261; // bit0 enable + bit5 ddr_en + bit6 push_en + bit9 pair_en
const TX_CMD_ARM_PAIR: u32 = 8;
const TX_CMD_ACK_PAIR: u32 = 16;
const TX_CMD_CANCEL_PAIR: u32 = 32;
const PAIR_SESSION: u32 = 0xA1B2_C3D4;
const MAILBOX_DBELL_A_OFF: u32 = 32;
const MAILBOX_DBELL_B_OFF: u32 = 36;


/// Linux EINTR
const EINTR: i32 = 4;

// 退出码（与 C 版 enum exit_code 一致）
const EX_OK: i32 = 0;
const EX_USAGE: i32 = 1;
const EX_DEV: i32 = 2;
const EX_NOT_READY: i32 = 3;
const EX_NO_FRAME: i32 = 4;
const EX_INCONSISTENT: i32 = 5;
const EX_OUTPUT: i32 = 6;

// 与驱动、FPGA 共用同一份布局：字段偏移算错就会取到错数据，编译期先钉死
// （drivers 侧 anlogic_pci_frame.c 里有同样的 static_assert；Rust 侧的断言在
//   src/ioctl.rs 里，等价于 C 版的这四条 _Static_assert）。
const _: () = {
    assert!(std::mem::size_of::<FrameMailbox>() == 64);
    assert!(std::mem::offset_of!(FrameMailbox, seq) == 4);
    assert!(std::mem::offset_of!(FrameMailbox, count) == 8);
    assert!(std::mem::offset_of!(FrameMailbox, stride) == 20);
    assert!(std::mem::size_of::<FrameSlotHdr>() == 16);
    assert!(std::mem::offset_of!(FrameSlotHdr, len) == 4);
    assert!(std::mem::offset_of!(FrameSlotHdr, flags) == 8);
};

fn usage(prog: &str) {
    eprint!(
        "用法：{} [-d 设备节点] [-t 超时ms] [-n] [-o 输出.pgm] [--info] [--pair] [--ctrl 节点] [--out-b B.pgm] [输出.pgm]\n\
         \x20 -d  收帧缓冲节点，默认 {}\n\
         \x20 -t  等帧超时（毫秒，1..600000），默认 {}\n\
         \x20 -n  只在出现更新的帧时返回（默认：缓冲里已有帧就直接取最新一帧）\n\
         \x20 -o  输出 PGM 文件名，默认 {}\n\
         \x20 --pair 双路同步取帧：A 写 -o，B 写 --out-b；需要 frame_slots=2\n\
         \x20 --ctrl 控制节点，默认 {}\n\
         \x20 --out-b 双路时 B 路 PGM，默认 {}\n\
         \x20 --info 只打印缓冲物理地址/几何/序号，不取帧\n\
         退出码：0 成功，2 设备不可用，3 描述符不符，4 无新帧，5 一致性不过，6 输出错误\n",
        prog, DEF_DEV, DEF_TIMEOUT_MS, DEF_OUT, DEF_CTRL, DEF_OUT_B
    );
}

/// 读 mailbox 的一个字段（mmap 上直接读；volatile + 全屏障，别让编译器合并/提前）
fn mb_field(p: *const u32) -> u32 {
    // SAFETY: 调用方保证地址 4 字节对齐且在映射内
    let v = unsafe { sys::rd32(p) };
    std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
    v
}

fn frame_info(fd: RawFd, info: &mut IocFrame) -> Result<(), ()> {
    *info = IocFrame::default();
    // SAFETY: info 就是 ANLOGIC_IOCFRAMEINFO 对应的结构体
    if let Err(e) = unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCFRAMEINFO, info as *mut IocFrame as *mut c_void) } {
        eprintln!("!! ANLOGIC_IOCFRAMEINFO 失败：{}", sys::errno_str(&e));
        return Err(());
    }
    Ok(())
}


fn bar_rw(fd: RawFd, off: u32, wdata: Option<u32>) -> Result<u32, std::io::Error> {
    let mut b = IocBar {
        bar_id: 0,
        bar_type: 0,
        bar_offaddr: off,
        wdata: wdata.unwrap_or(0),
        rdata: 0,
    };
    let req = if wdata.is_some() { ANLOGIC_IOCW } else { ANLOGIC_IOCR };
    // SAFETY: b 就是 anlogic_ioc_bar，编号与驱动头文件一致
    unsafe {
        sys::ioctl_ptr(
            fd,
            req,
            &mut b as *mut IocBar as *mut c_void,
        )?;
    }
    Ok(b.rdata)
}

fn bar_wr(fd: RawFd, off: u32, v: u32) -> Result<(), String> {
    bar_rw(fd, off, Some(v))
        .map(|_| ())
        .map_err(|e| format!("写 BAR 0x{off:03x}=0x{v:08x} 失败：{}", sys::errno_str(&e)))
}

fn bar_rd(fd: RawFd, off: u32) -> Result<u32, String> {
    bar_rw(fd, off, None)
        .map_err(|e| format!("读 BAR 0x{off:03x} 失败：{}", sys::errno_str(&e)))
}

fn wr64(lo_off: u32, hi_off: u32, v: u64, fd: RawFd) -> Result<(), String> {
    bar_wr(fd, lo_off, v as u32)?;
    bar_wr(fd, hi_off, (v >> 32) as u32)
}

/// 等一个 32 位门铃从 old 变成新值；超时返回 None。
fn wait_dbell(map: &Mmap, off: usize, old: u32, timeout_ms: u32) -> Option<u32> {
    let p = (map.as_ptr() as usize + off) as *const u32;
    let t0 = std::time::Instant::now();
    loop {
        let v = mb_field(p);
        if v != old {
            return Some(v);
        }
        if t0.elapsed() >= std::time::Duration::from_millis(timeout_ms as u64) {
            return None;
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

/// 描述符自检：驱动刚分配完就会把几何写进 mailbox（magic 最后写），
/// 这里既核对 ioctl 报的，也核对 mmap 里读到的首部，免得两边对不上还去算地址。
fn descriptor_check(info: &IocFrame) -> bool {
    if info.phys_addr == 0
        || info.size == 0
        || info.slots == 0
        || info.slot_bytes == 0
        || info.stride == 0
        || info.hdr_bytes == 0
        || info.width == 0
        || info.height == 0
        || info.mb_off == 0
    {
        eprintln!(
            "!! 驱动报的收帧缓冲描述符不完整：phys=0x{:x} size={} stride={} slot={} hdr={} slots={} {}x{} mb_off={}",
            info.phys_addr, info.size, info.stride, info.slot_bytes, info.hdr_bytes,
            info.slots, info.width, info.height, info.mb_off
        );
        return false;
    }
    if (info.stride as u64) < info.hdr_bytes as u64 + info.slot_bytes as u64 {
        eprintln!(
            "!! stride {} < 槽头 {} + 一帧 {}：布局对不上",
            info.stride, info.hdr_bytes, info.slot_bytes
        );
        return false;
    }
    // 槽区 [0, stride*slots) 必须装得下所有槽，且不能压到 mailbox
    if info.size < info.stride as u64 * info.slots as u64 {
        eprintln!(
            "!! 缓冲 {} 字节装不下 {} 槽 × 步长 {}",
            info.size, info.slots, info.stride
        );
        return false;
    }
    if info.mb_off & 0xFFF != 0 {
        eprintln!("!! mb_off {} 不是页对齐：mailbox 得独占一页", info.mb_off);
        return false;
    }
    if (info.mb_off as u64) < info.stride as u64 * info.slots as u64 {
        eprintln!(
            "!! mb_off {} 落在槽区（{} 槽 × 步长 {}）里：会互相覆盖",
            info.mb_off, info.slots, info.stride
        );
        return false;
    }
    if info.mb_off as u64 + std::mem::size_of::<FrameMailbox>() as u64 > info.size {
        eprintln!("!! mailbox 偏移 {} + 64 字节超出缓冲 {}", info.mb_off, info.size);
        return false;
    }
    // 槽长必须与几何推导一致：align4k(128B R10F 头 + width/4*5 * height)。
    // 1080p = 2,592,768；4K = 10,371,072。驱动按 module param 现算，这里复核。
    if info.width % 4 == 0 {
        let payload = (info.width as u64 / 4 * 5) * info.height as u64;
        let expect_slot = (payload + r10f::HDR_BYTES as u64 + 4095) & !4095u64;
        if expect_slot != info.slot_bytes as u64 {
            eprintln!(
                "!! 槽长 {} 与 {}x{} 的 R10F 推导值 {} 不一致（128B 头+packed RAW10，4KiB 对齐）",
                info.slot_bytes, info.width, info.height, expect_slot
            );
            return false;
        }
    }
    if info.slot_bytes < info.width * info.height {
        eprintln!(
            "!! 警告：一帧只有 {} 字节，小于 {}x{} 灰度需要的 {}",
            info.slot_bytes,
            info.width,
            info.height,
            info.width * info.height
        );
    }
    true
}

fn print_info(info: &IocFrame) {
    println!("=== 收帧缓冲 ===");
    println!("  物理地址（FPGA 的 MWr 基址）: 0x{:x}", info.phys_addr);
    println!(
        "  缓冲大小                    : {} 字节（{} KiB，页对齐）",
        info.size,
        info.size / 1024
    );
    println!(
        "  槽位                        : {} 槽，步长 {} 字节，槽内数据偏移 {} 字节（槽 n 在 phys + {}×n）",
        info.slots, info.stride, info.hdr_bytes, info.stride
    );
    println!(
        "  mailbox                     : 缓冲内偏移 {}（独占一页，FPGA 把 seq/count 写这里）",
        info.mb_off
    );
    println!(
        "  每帧传输                    : {}x{} R10F = {} 字节（槽内容，不是灰度像素数）",
        info.width, info.height, info.slot_bytes
    );
    println!(
        "  序号/计数（驱动看门铃补齐） : seq={} count={}{}",
        info.seq,
        info.count,
        if info.seq != 0 { "" } else { "（seq 还是 0：还没有推来的帧）" }
    );
}

/// packed RAW10（MIPI CSI-2，4 像素/5 字节）→ 8bit 灰度预览。
///
/// 字节序与 R10F 合同（`docs/protocol/rka1-r10f-v1/`）及
/// `docs/protocol/rka1-r10f-v1/tools/r10f_strict.py` 一致：每组 `P0[9:2] P1[9:2] P2[9:2] P3[9:2]
/// {P3[1:0],P2[1:0],P1[1:0],P0[1:0]}`。预览取每个 10bit 采样的**高 8 位**（`>> 2`，
/// 截断不四舍五入；也就是每组前 4 个字节）。
///
/// `stride` 是 R10F 头里的**行字节数**（1080p 为 2400，4K 为 4800）；行尾填充不会进图。
/// 返回恰好 `width*height` 字节；长度/几何不符一律 `Err`，不越界读、不静默夹紧。
fn raw10_to_gray8(
    packed: &[u8],
    width: usize,
    height: usize,
    stride: usize,
) -> Result<Vec<u8>, String> {
    if width == 0 || height == 0 {
        return Err(format!("几何 {}x{} 非法", width, height));
    }
    if width % 4 != 0 {
        return Err(format!("width {} 不是 4 的倍数（packed RAW10 是 4px/5B）", width));
    }
    let row_bytes = width / 4 * 5;
    if stride < row_bytes {
        return Err(format!(
            "stride {} 装不下一行 packed RAW10（需要 {} 字节）",
            stride, row_bytes
        ));
    }
    // 最后一行只需要 row_bytes：行尾 padding（例如 4KiB 对齐）不读
    let need = stride
        .checked_mul(height - 1)
        .and_then(|v| v.checked_add(row_bytes))
        .ok_or_else(|| "packed 长度计算溢出".to_string())?;
    if packed.len() < need {
        return Err(format!(
            "packed 载荷 {} 字节不够 {} 行 × stride {}（至少 {} 字节）",
            packed.len(),
            height,
            stride,
            need
        ));
    }
    let npix = width
        .checked_mul(height)
        .ok_or_else(|| "像素数溢出".to_string())?;
    let mut out = Vec::new();
    out.try_reserve_exact(npix)
        .map_err(|e| format!("分配 {} 字节预览失败：{}", npix, e))?;
    out.resize(npix, 0u8);
    for y in 0..height {
        let src = &packed[y * stride..y * stride + row_bytes];
        let dst = &mut out[y * width..(y + 1) * width];
        for (g, d) in src.chunks_exact(5).zip(dst.chunks_exact_mut(4)) {
            let b4 = g[4] as u16;
            let p0 = ((g[0] as u16) << 2) | (b4 & 0x3);
            let p1 = ((g[1] as u16) << 2) | ((b4 >> 2) & 0x3);
            let p2 = ((g[2] as u16) << 2) | ((b4 >> 4) & 0x3);
            let p3 = ((g[3] as u16) << 2) | ((b4 >> 6) & 0x3);
            d[0] = (p0 >> 2) as u8;
            d[1] = (p1 >> 2) as u8;
            d[2] = (p2 >> 2) as u8;
            d[3] = (p3 >> 2) as u8;
        }
    }
    Ok(out)
}

/// 槽内容若是 R10F：严格解析（[`r10f::parse`]）并解包成 8bit 预览。
///
/// · `Ok(None)`：不是 R10F（旧 8bit 槽内容），调用方走原来的直拷路径；
/// · `Ok(Some((头, 预览)))`：R10F 通过严格校验，预览恰好 `width*height` 字节；
/// · `Err`：确实是 R10F 但头/CRC/尾零/几何/长度不过 —— 一律拒绝发布。
fn r10f_gray8(
    content: &[u8],
    expect_w: u32,
    expect_h: u32,
) -> Result<Option<(r10f::R10fHeader, Vec<u8>)>, String> {
    if content.len() < r10f::HDR_BYTES || &content[0..4] != b"R10F" {
        return Ok(None);
    }
    if expect_w > u16::MAX as u32 || expect_h > u16::MAX as u32 {
        return Err(format!(
            "已授权几何 {}x{} 超出 R10F 头的 u16 字段，不可能匹配",
            expect_w, expect_h
        ));
    }
    let pb = r10f::le32(content, 0x30) as usize;
    let need = r10f::HDR_BYTES
        .checked_add(pb)
        .ok_or_else(|| format!("payload_bytes={} 溢出", pb))?;
    if need > content.len() {
        return Err(format!(
            "payload_bytes={} 超出槽内容（{} 字节）：拒绝发布",
            pb,
            content.len()
        ));
    }
    let (hdr, rest) = content.split_at(r10f::HDR_BYTES);
    let (payload, tail) = rest.split_at(pb);
    let t0 = std::time::Instant::now();
    let h = r10f::parse(hdr, payload, tail, expect_w as u16, expect_h as u16, None)?;
    let t1 = std::time::Instant::now();
    let preview = raw10_to_gray8(payload, h.width as usize, h.height as usize, h.stride as usize)?;
    if std::env::var_os("FRAME_DEBUG_TIMING").is_some() {
        eprintln!("  [timing] parse {:?}, decode {:?}", t1.duration_since(t0), t1.elapsed());
    }
    Ok(Some((h, preview)))
}


fn write_pgm(path: &str, width: u32, height: u32, gray: &[u8]) -> Result<(), String> {
    let mut fp = std::fs::File::create(path)
        .map_err(|e| format!("打不开输出文件 {path}：{}", sys::errno_str(&e)))?;
    let header = format!("P5\n{width} {height}\n255\n");
    fp.write_all(header.as_bytes())
        .and_then(|_| fp.write_all(gray))
        .and_then(|_| fp.flush())
        .map_err(|e| format!("写 {path} 失败：{}", sys::errno_str(&e)))
}

fn pair_run(
    map: &Mmap,
    info: &IocFrame,
    ctrl_path: &str,
    out_a: &str,
    out_b: &str,
    timeout_ms: u32,
) -> i32 {
    if info.slots < 2 {
        eprintln!("!! --pair 需要 frame_slots=2（当前 {}）：先 rmmod/insmod 驱动", info.slots);
        return EX_NOT_READY;
    }
    let ctrl = match std::fs::OpenOptions::new().read(true).write(true).open(ctrl_path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("!! 打不开控制节点 {ctrl_path}：{}", sys::errno_str(&e));
            return EX_DEV;
        }
    };
    let cfd = ctrl.as_raw_fd();

    // 清掉可能残留的 single/pair one-shot；只发一次取消，等 20ms 让状态机回到 IDLE。
    // 不能把 0xC4 的 bit2（db_sent）当成 held 轮询，否则每轮都要空等 500ms。
    let _ = bar_wr(cfd, BAR_TX_CMD, TX_CMD_CANCEL_PAIR);
    let _ = bar_wr(cfd, BAR_TX_CMD, 4);
    std::thread::sleep(std::time::Duration::from_millis(20));

    // 模式位：enable + ddr_en + push_en + pair_en；清掉 CAM_B/PUSH_SRC。
    let ctrl_v = (bar_rd(cfd, BAR_CTRL).unwrap_or(0) | CTRL_PAIR_BITS) & !0x180;
    if let Err(e) = bar_wr(cfd, BAR_CTRL, ctrl_v) {
        eprintln!("!! {e}");
        return EX_DEV;
    }
    let cap = info.stride * 2;
    let dst_a = info.phys_addr + info.hdr_bytes as u64;
    let dst_b = info.phys_addr + info.stride as u64 + info.hdr_bytes as u64;
    let db_a = info.phys_addr + info.mb_off as u64 + MAILBOX_DBELL_A_OFF as u64;
    let db_b = info.phys_addr + info.mb_off as u64 + MAILBOX_DBELL_B_OFF as u64;
    let cfg = (|| -> Result<(), String> {
        bar_wr(cfd, BAR_TX_CAP, cap)?;
        bar_wr(cfd, BAR_TX_SESSION, PAIR_SESSION)?;
        // ACK_RELEASE 比对的是 TX_ACK_SESSION（0x110），不是 SESSION_ID（0x104）。
        // 只写 0x104 会让 ACK 永远 BAD_ACK，两片 pusher 停在 HELD。
        bar_wr(cfd, 0x110, PAIR_SESSION)?;
        wr64(BAR_PUSH_DST_LO, BAR_PUSH_DST_HI, dst_a, cfd)?;
        wr64(BAR_PUSH_DB_LO, BAR_PUSH_DB_HI, db_a, cfd)?;
        wr64(BAR_PAIR_DST_LO, BAR_PAIR_DST_HI, dst_b, cfd)?;
        wr64(BAR_PAIR_DB_LO, BAR_PAIR_DB_HI, db_b, cfd)?;
        bar_wr(cfd, BAR_TX_ACK_SEQ, 0)?;
        bar_wr(cfd, BAR_PAIR_ACK_SEQ, 0)?;
        Ok(())
    })();
    if let Err(e) = cfg {
        eprintln!("!! {e}");
        return EX_DEV;
    }

    // ARM 前必须等两路 DDR 都发布过完整帧：RTL 的 arm_reject 要求 ddr_status[0]=1。
    let mut ddr_ok = false;
    for _ in 0..(timeout_ms / 2).max(1) {
        let sa = bar_rd(cfd, 0xA8).unwrap_or(0);
        let sb = bar_rd(cfd, 0xE8).unwrap_or(0);
        if sa & 1 != 0 && sb & 1 != 0 {
            ddr_ok = true;
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    if !ddr_ok {
        let sa = bar_rd(cfd, 0xA8).unwrap_or(0);
        let sb = bar_rd(cfd, 0xE8).unwrap_or(0);
        eprintln!("!! 两路 DDR 还没发布完整帧（A=0x{sa:x} B=0x{sb:x}），不 ARM");
        return EX_NO_FRAME;
    }

    let sync_pairs0 = bar_rd(cfd, 0x1EC).unwrap_or(0);
    let old_a = mb_field((map.as_ptr() as usize + info.mb_off as usize + MAILBOX_DBELL_A_OFF as usize) as *const u32);
    let old_b = mb_field((map.as_ptr() as usize + info.mb_off as usize + MAILBOX_DBELL_B_OFF as usize) as *const u32);
    let t_cfg = std::time::Instant::now();
    println!("=== 双路同步取帧（A 槽 0 / B 槽 1，超时 {timeout_ms} ms）===");
    println!("  A dst=0x{dst_a:x} db=0x{db_a:x}  B dst=0x{dst_b:x} db=0x{db_b:x}");
    if let Err(e) = bar_wr(cfd, BAR_TX_CMD, TX_CMD_ARM_PAIR) {
        eprintln!("!! {e}");
        return EX_DEV;
    }
    std::thread::sleep(std::time::Duration::from_millis(2));
    let tx_a = bar_rd(cfd, 0x114).unwrap_or(0);
    if tx_a & 0x8 != 0 {
        eprintln!("!! ARM_PAIR 被拒（A TX_STATUS=0x{tx_a:x}）：检查 DDR/session/cap");
        return EX_DEV;
    }

    let db_a_off = info.mb_off as usize + MAILBOX_DBELL_A_OFF as usize;
    let db_b_off = info.mb_off as usize + MAILBOX_DBELL_B_OFF as usize;
    let new_a = wait_dbell(map, db_a_off, old_a, timeout_ms);
    let new_b = wait_dbell(map, db_b_off, old_b, timeout_ms);
    let (Some(new_a), Some(new_b)) = (new_a, new_b) else {
        let sa = bar_rd(cfd, BAR_PUSH_STATUS).unwrap_or(0);
        let sb = bar_rd(cfd, BAR_PAIR_STATUS).unwrap_or(0);
        let ea = bar_rd(cfd, BAR_PUSH_ERR).unwrap_or(0xff);
        let eb = bar_rd(cfd, BAR_PAIR_ERR).unwrap_or(0xff);
        eprintln!("!! 双路等门铃超时：A {old_a}->{:?} B {old_b}->{:?} status A=0x{sa:x} B=0x{sb:x} err A={ea} B={eb}",
                  new_a, new_b);
        return EX_NO_FRAME;
    };
    let t_done = std::time::Instant::now();
    let sync_pairs1 = bar_rd(cfd, 0x1EC).unwrap_or(sync_pairs0);
    println!("  门铃：A {old_a}->{new_a}，B {old_b}->{new_b}；sync pair_count {sync_pairs0}->{sync_pairs1}");

    println!("  时间：配置+等门铃 {:?}", t_done.duration_since(t_cfg));
    let t_parse = std::time::Instant::now();
    // 两槽的 R10F 内容直接从 mmap 读；槽头由驱动补 A 路，B 路不依赖槽头。
    let mut raws = Vec::with_capacity(2);
    for slot in 0..2u32 {
        let off = slot as u64 * info.stride as u64 + info.hdr_bytes as u64;
        // 先搬进普通内存再验 CRC：驱动这块 DMA 是 Normal-NC，直接对 mmap
        // 做 2.6MB 的 CRC 会跑在非缓存访问上（实测一帧 ~350ms）。
        let mut raw = vec![0u8; info.slot_bytes as usize];
        unsafe {
            std::ptr::copy_nonoverlapping(
                (map.as_ptr() as usize + off as usize) as *const u8,
                raw.as_mut_ptr(),
                raw.len(),
            );
        }
        match r10f_gray8(&raw, info.width, info.height) {
            Ok(Some((h, preview))) => {
                println!("  槽 {slot}: R10F session=0x{:08x} seq={} {}x{} stride={}",
                         h.session_id, h.capture_seq, h.width, h.height, h.stride);
                raws.push(preview);
            }
            Ok(None) => {
                eprintln!("!! 槽 {slot} 不是 R10F：拒绝发布");
                return EX_INCONSISTENT;
            }
            Err(e) => {
                eprintln!("!! 槽 {slot} R10F 校验失败：{e}");
                return EX_INCONSISTENT;
            }
        }
    }
    println!("  时间：两帧解析+解包 {:?}", t_parse.elapsed());
    if let Err(e) = write_pgm(out_a, info.width, info.height, &raws[0]) {
        eprintln!("!! {e}");
        return EX_OUTPUT;
    }
    if let Err(e) = write_pgm(out_b, info.width, info.height, &raws[1]) {
        eprintln!("!! {e}");
        return EX_OUTPUT;
    }

    // ACK 两路：A 用 0x10C，B 用 0x220；随后 pair ACK 同时释放两片。
    let done_a = bar_rd(cfd, BAR_PUSH_DONE).unwrap_or(new_a);
    let done_b = bar_rd(cfd, BAR_PAIR_DONE).unwrap_or(new_b);
    let ack = (|| -> Result<(), String> {
        bar_wr(cfd, BAR_TX_ACK_SEQ, done_a)?;
        bar_wr(cfd, BAR_PAIR_ACK_SEQ, done_b)?;
        bar_wr(cfd, BAR_TX_CMD, TX_CMD_ACK_PAIR)
    })();
    if let Err(e) = ack {
        eprintln!("!! {e}（图已写出；两路可能停在 HELD）");
        return EX_DEV;
    }
    // 门铃先于 g_st→HELD，刚发完 ACK 时 B 可能还没进 HELD；补一拍 CANCEL 兜底。
    // 主机数据已拷到槽里，CANCEL 只释放源块，不会改已收到的两帧。
    let _ = bar_wr(cfd, BAR_TX_CMD, TX_CMD_CANCEL_PAIR);
    std::thread::sleep(std::time::Duration::from_millis(100));
    // ACK 释放可能晚于 20ms 才在状态字上稳定；轮询到两片都退出 HELD。
    let mut ps = 0xffu32;
    for _ in 0..500 {
        ps = bar_rd(cfd, BAR_PAIR_STATUS).unwrap_or(0xff);
        if ps & 0x3 == 0 {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    println!("  ACK：A seq={done_a} B seq={done_b}；status A=0x{:x} B=0x{:x} pair=0x{:x}",
             bar_rd(cfd, BAR_PUSH_STATUS).unwrap_or(0),
             bar_rd(cfd, BAR_PAIR_STATUS).unwrap_or(0), ps);
    println!("  输出：{out_a}（A）、{out_b}（B）");
    EX_OK
}

fn run() -> i32 {
    let argv: Vec<String> = std::env::args_os()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();

    let longopts = [
        LongOpt { name: "dev", has_arg: true, val: b'd' as u32 },
        LongOpt { name: "timeout", has_arg: true, val: b't' as u32 },
        LongOpt { name: "new", has_arg: false, val: b'n' as u32 },
        LongOpt { name: "output", has_arg: true, val: b'o' as u32 },
        LongOpt { name: "info", has_arg: false, val: 1 },
        LongOpt { name: "pair", has_arg: false, val: 2 },
        LongOpt { name: "ctrl", has_arg: true, val: 3 },
        LongOpt { name: "out-b", has_arg: true, val: 4 },
        LongOpt { name: "help", has_arg: false, val: b'h' as u32 },
    ];

    let mut getopt = Getopt::new(&argv, "d:t:no:h", &longopts);

    let mut dev_path = DEF_DEV.to_string();
    let mut ctrl_path = DEF_CTRL.to_string();
    let mut out_path: Option<String> = None;
    let mut out_b_path = DEF_OUT_B.to_string();
    let mut timeout_ms: u32 = DEF_TIMEOUT_MS;
    let mut wait_new = false;
    let mut want_info = false;
    let mut want_pair = false;

    while let Some(item) = getopt.next() {
        let (id, arg) = match item {
            Ok(o) => o,
            Err(msg) => {
                eprintln!("{}: {}", argv[0], msg);
                usage(&argv[0]);
                return EX_USAGE;
            }
        };
        match id as u8 {
            b'd' => dev_path = arg.unwrap_or_default(),
            b't' => {
                let s = arg.unwrap_or_default();
                match strtoul_base0(&s) {
                    Some(v) if v != 0 && v <= ANLOGIC_FRAME_MAX_WAIT_MS as u64 => timeout_ms = v as u32,
                    _ => {
                        eprintln!("!! --timeout 需要 1..{} 的毫秒数（收到 '{}'）", ANLOGIC_FRAME_MAX_WAIT_MS, s);
                        return EX_USAGE;
                    }
                }
            }
            b'n' => wait_new = true,
            b'o' => out_path = Some(arg.unwrap_or_default()),
            1 => want_info = true,
            2 => want_pair = true,
            3 => ctrl_path = arg.unwrap_or_default(),
            4 => out_b_path = arg.unwrap_or_default(),
            b'h' => {
                usage(&argv[0]);
                return EX_OK;
            }
            _ => {
                usage(&argv[0]);
                return EX_USAGE;
            }
        }
    }
    for a in getopt.positional() {
        if out_path.is_none() {
            out_path = Some(a);
        } else if want_pair && out_b_path == DEF_OUT_B {
            out_b_path = a;
        } else {
            eprintln!("!! 多余的参数：'{}'", a);
            return EX_USAGE;
        }
    }
    let out_path = out_path.unwrap_or_else(|| DEF_OUT.to_string());

    // ---- 1. 打开节点（只读）并独占 ----
    let file = match std::fs::OpenOptions::new().read(true).open(&dev_path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("!! 打不开 {}：{}", dev_path, sys::errno_str(&e));
            eprintln!("   （需要 root；驱动已 insmod 且设备已枚举；节点名形如 /dev/ANLOGIC-PCI0_frame）");
            return EX_DEV;
        }
    };
    let fd = file.as_raw_fd();
    if let Err(e) = sys::flock_exclusive_nb(fd) {
        eprintln!("!! {} 已被其它进程占用（flock 失败：{}）", dev_path, sys::errno_str(&e));
        return EX_DEV;
    }

    // ---- 2. 问缓冲物理地址 / 大小 / 序号 ----
    let mut info = IocFrame::default();
    if frame_info(fd, &mut info).is_err() {
        return EX_DEV;
    }
    if !descriptor_check(&info) {
        return EX_NOT_READY;
    }
    print_info(&info);

    if want_info {
        return EX_OK;
    }

    // ---- 3. 只读映射整块缓冲，并核对描述符首部 ----
    let map = match Mmap::shared(fd, info.size as usize, PROT_READ) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("!! mmap {} 字节失败：{}", info.size, sys::errno_str(&e));
            return EX_DEV;
        }
    };
    // 定位 mailbox：偏移由驱动报的 mb_off 给出，不自己按常量猜
    let mb = (map.as_ptr() as usize + info.mb_off as usize) as *const FrameMailbox;
    if mb_field(mb as *const u32) != ANLOGIC_FRAME_MAGIC {
        eprintln!(
            "!! mailbox（偏移 {}）的 magic 不是 0x{:08x}：驱动还没把描述符写好，先别取帧",
            info.mb_off, ANLOGIC_FRAME_MAGIC
        );
        return EX_NOT_READY;
    }

    if want_pair {
        return pair_run(&map, &info, &ctrl_path, &out_path, &out_b_path, timeout_ms);
    }

    // ---- 4. 等帧：默认只要缓冲里有帧就取最新那帧；-n 则等一帧更新的 ----
    let mut wait = IocFrameWait::default();
    wait.last_seq = if wait_new { info.seq } else { 0 };
    wait.timeout_ms = timeout_ms;
    println!("=== 等帧（轮询序号，超时 {} ms，last_seq={}）===", timeout_ms, wait.last_seq);
    // SAFETY: wait 就是 ANLOGIC_IOCFRAMEWAIT 对应的结构体
    if let Err(e) = unsafe {
        sys::ioctl_ptr(fd, ANLOGIC_IOCFRAMEWAIT, &mut wait as *mut IocFrameWait as *mut c_void)
    } {
        if e.raw_os_error() == Some(EINTR) {
            eprintln!("!! 等帧被信号打断");
        } else {
            eprintln!("!! ANLOGIC_IOCFRAMEWAIT 失败：{}", sys::errno_str(&e));
        }
        return EX_DEV;
    }
    if wait.ready == 0 {
        println!(
            "无新帧：seq={}（last_seq={}），count={} —— FPGA 还没把帧写进这块缓冲，本次不写文件",
            wait.seq, wait.last_seq, wait.count
        );
        return EX_NO_FRAME;
    }

    // ---- 5. 定位槽位并核对槽头（顺序：先看 seq，再看 len/flags）----
    let slot = wait.slot;
    if slot >= info.slots {
        eprintln!("!! 驱动报的槽位 {} 越界（共 {} 槽）", slot, info.slots);
        return EX_INCONSISTENT;
    }
    let hdr = (map.as_ptr() as usize + slot as usize * info.stride as usize) as *const FrameSlotHdr;
    let hdr_seq = mb_field(hdr as *const u32);
    let hdr_len = mb_field(unsafe { hdr.cast::<u32>().add(1) });
    let hdr_flags = mb_field(unsafe { hdr.cast::<u32>().add(2) });
    println!(
        "  新帧：seq={}（count={}），槽位 {}，槽头 {{seq={}, len={}, flags=0x{:x}}}",
        wait.seq, wait.count, slot, hdr_seq, hdr_len, hdr_flags
    );

    let mut rc = EX_OK;
    if hdr_seq != wait.seq {
        eprintln!(
            "!! 槽头序号 {} 与 mailbox.seq {} 不一致（FPGA 还没按约定先写槽头再改序号？）",
            hdr_seq, wait.seq
        );
        rc = EX_INCONSISTENT;
    }
    if hdr_flags & ANLOGIC_FRAME_FLAG_COMPLETE == 0 {
        eprintln!(
            "!! 槽头 flags=0x{:x} 没标 COMPLETE（bit0）：这一槽可能还在被写",
            hdr_flags
        );
        rc = EX_INCONSISTENT;
    }

    // ---- 6. 搬槽内容（长度 = slot_bytes，不是 width*height）----
    /*
     * T07 R10F 的槽内容 = 128B 头 + packed RAW10（5B/4px）+ 4KiB 对齐零尾：
     * 1080p 是 2,592,768 字节、4K 是 10,371,072 字节，而 4K 的 width*height
     * 只有 8,294,400（灰度预览）。
     * 旧代码按 npix 分配、却按槽头 len（= slot_bytes）拷贝，两者不等就写穿堆。
     * 现在**分配长度 = 拷贝长度 = slot_bytes**，再把「槽基址 + 槽头 + 内容」钉在映射内。
     * width/height 来自 module param，最大分别可到 65535；分配失败一律按 EX_OUTPUT 返回
     * （直接 vec! 在失败时是 abort，退出码 134，不在文件头声明的 0..6 里）。
     */
    let npix = info.width as usize * info.height as usize;
    let copy = hdr_len as usize;
    /*
     * T07: no silent clamping.  The slot length is the transport length agreed
     * with the FPGA (dma_bytes); anything else means host and FPGA disagree,
     * and a clamped copy would hand out a frame that looks fine but is not.
     */
    if copy != info.slot_bytes as usize {
        eprintln!(
            "!! 槽头 len={} != 驱动/fpga 约定的 slot_bytes={}：拒绝发布（不再夹紧）",
            hdr_len, info.slot_bytes
        );
        return EX_INCONSISTENT;
    }
    if copy & 0xfff != 0 {
        eprintln!("!! slot_bytes={} 不是 4KiB 整数倍：不是本框架的传输格式", copy);
        return EX_INCONSISTENT;
    }
    let src_off = slot as u64 * info.stride as u64 + info.hdr_bytes as u64;
    if src_off + copy as u64 > info.size {
        eprintln!(
            "!! 槽内容 [{}, {}) 超出缓冲 {} 字节：拒绝读（会越界）",
            src_off,
            src_off + copy as u64,
            info.size
        );
        return EX_INCONSISTENT;
    }
    let mut raw: Vec<u8> = Vec::new();
    if raw.try_reserve_exact(copy).is_err() {
        eprintln!("!! 分配 {} 字节槽内容缓冲失败（内存不足）", copy);
        return EX_OUTPUT;
    }
    raw.resize(copy, 0u8);
    let src = map.as_ptr() as usize + src_off as usize;
    // SAFETY: 上面已核对 [src_off, src_off+copy) 落在 mmap 的 [0, info.size) 内
    unsafe {
        std::ptr::copy_nonoverlapping(src as *const u8, raw.as_mut_ptr(), copy);
    }

    // ---- 6.5 按内容分派：R10F → 严格解析 + 解包预览；否则旧 8bit 直拷 ----
    //   判据与 `docs/protocol/rka1-r10f-v1/tools/r10f_strict.py` / contract/acceptance-vectors.json
    //   逐字段一致（magic/version/format/codec/几何三层长度/三 CRC/尾零/flags）。
    //   ⚠ R10F 的载荷是 packed RAW10（4px/5B），不是灰度像素：旧实现把它原样写进 PGM
    //   （图是错的）；现在解成 width*height 的 8bit 预览（每采样高 8 位）再写。
    let gray: Cow<'_, [u8]>;
    match r10f_gray8(&raw, info.width, info.height) {
        Ok(Some((h, preview))) => {
            println!(
                "  R10F      : session=0x{:08x} seq={} {}x{} stride={} flags=0x{:x}",
                h.session_id, h.capture_seq, h.width, h.height, h.stride, h.flags
            );
            println!(
                "  R10F CRC  : payload=0x{:08x} header=0x{:08x}（parse 内部已逐字节复算通过）",
                h.payload_crc32, h.header_crc32
            );
            println!(
                "  R10F 预览 : packed {}B → 8bit {}x{} = {}B（每采样取高 8 位 >>2）",
                h.payload_bytes, h.width, h.height, preview.len()
            );
            gray = Cow::Owned(preview);
        }
        Ok(None) => {
            // 旧 8bit 路径：槽内容前 width*height 字节就是像素；不足补 0 并告警。
            if copy < npix {
                eprintln!("!! 帧数据只有 {}/{} 字节，缺的用 0 补（图不完整）", copy, npix);
                rc = EX_INCONSISTENT;
                let mut v = raw.clone();
                v.resize(npix, 0);
                gray = Cow::Owned(v);
            } else {
                gray = Cow::Borrowed(&raw[..npix]);
            }
        }
        Err(e) => {
            eprintln!(
                "!! R10F 严格解析/解包失败：{}（拒绝发布；槽头编号/长度一致也无效）",
                e
            );
            return EX_INCONSISTENT;
        }
    }

    // ---- 7. 搬运转查：只有环形缓冲绕回一圈才可能盖掉刚取的这一帧 ----
    // （与 C 版一样重新问一次，覆盖同一个 info）
    if frame_info(fd, &mut info).is_err() {
        return EX_DEV;
    }
    let seq2 = info.seq;
    if seq2 != wait.seq {
        let advanced = seq2.wrapping_sub(wait.seq);
        println!(
            "  搬运期间序号 {} → {}（新出 {} 帧，环上一共 {} 槽）{}",
            wait.seq,
            seq2,
            advanced,
            info.slots,
            if advanced >= info.slots {
                " ← 已经绕回，这一槽可能被覆盖过！"
            } else {
                "（还没绕回，本帧没被覆盖）"
            }
        );
        if advanced >= info.slots {
            rc = EX_INCONSISTENT;
        }
    }

    // ---- 8. 写 P5 灰度 PGM ----
    let mut fp = match std::fs::File::create(&out_path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("!! 打不开输出文件 {}：{}", out_path, sys::errno_str(&e));
            return EX_OUTPUT;
        }
    };
    let header = format!("P5\n{} {}\n255\n", info.width, info.height);
    let written = fp
        .write_all(header.as_bytes())
        .and_then(|_| fp.write_all(&gray))
        .and_then(|_| fp.flush());
    if let Err(e) = written {
        eprintln!("!! 写 {} 失败：{}", out_path, sys::errno_str(&e));
        return EX_OUTPUT;
    }

    println!("=== 结果 ===");
    println!("  输出文件  : {}（P5 灰度 {}x{}，{} 字节像素）", out_path, info.width, info.height, npix);
    println!(
        "  取自      : 物理 0x{:x} + 槽 {}（偏移 {}）+ 数据偏移 {}",
        info.phys_addr,
        slot,
        slot as u64 * info.stride as u64,
        info.hdr_bytes
    );
    println!("  帧序号    : {}（累计 {} 帧）", wait.seq, wait.count);
    println!(
        "  一致性    : {}",
        if rc == EX_OK {
            "OK（槽头序号与 mailbox 一致、flags 标了 COMPLETE、搬运期间没绕回）"
        } else {
            "**不通过**（图已写出，但请勿当可信帧用；原因见上面的告警）"
        }
    );

    rc
}

fn main() {
    let code = run();
    anlogic_pci_tools::exit_with(code);
}

#[cfg(test)]
mod tests {
    use super::{descriptor_check, r10f_gray8, raw10_to_gray8};
    use anlogic_pci_tools::ioctl::IocFrame;
    use anlogic_pci_tools::rka1::r10f;

    /// 4 个 10bit 样本 → 5 字节（与 R10F 合同及
    /// `docs/protocol/rka1-r10f-v1/tools/r10f_strict.py` 的严格校验同字节序）
    fn pack_group(p: [u16; 4]) -> [u8; 5] {
        [
            (p[0] >> 2) as u8,
            (p[1] >> 2) as u8,
            (p[2] >> 2) as u8,
            (p[3] >> 2) as u8,
            (((p[3] & 3) << 6) | ((p[2] & 3) << 4) | ((p[1] & 3) << 2) | (p[0] & 3)) as u8,
        ]
    }

    /// 造一帧小的合法 R10F：128B 头 + payload + 尾零（CRC 按合同算法现算）
    fn synthetic_r10f(payload: &[u8], w: u16, h: u16, stride: u32, tail: &[u8]) -> Vec<u8> {
        let mut hdr = vec![0u8; r10f::HDR_BYTES];
        hdr[0..4].copy_from_slice(b"R10F");
        hdr[0x04..0x06].copy_from_slice(&1u16.to_le_bytes()); // version 1.0
        hdr[0x06..0x08].copy_from_slice(&0u16.to_le_bytes());
        hdr[0x08..0x0A].copy_from_slice(&(r10f::HDR_BYTES as u16).to_le_bytes());
        hdr[0x0B] = r10f::FORMAT_RAW10_PACKED as u8;
        hdr[0x10..0x14].copy_from_slice(&0x2026_1027u32.to_le_bytes()); // session
        hdr[0x14..0x18].copy_from_slice(&7u32.to_le_bytes()); // capture_seq
        hdr[0x18..0x1C].copy_from_slice(&1u32.to_le_bytes()); // mode_id
        hdr[0x1C..0x1E].copy_from_slice(&w.to_le_bytes());
        hdr[0x1E..0x20].copy_from_slice(&h.to_le_bytes());
        hdr[0x20..0x24].copy_from_slice(&stride.to_le_bytes());
        hdr[0x2A] = r10f::VALID_BITS;
        hdr[0x2C..0x30].copy_from_slice(&(payload.len() as u32).to_le_bytes()); // raw_bytes
        hdr[0x30..0x34].copy_from_slice(&(payload.len() as u32).to_le_bytes()); // payload_bytes
        hdr[0x34..0x38].copy_from_slice(&1u32.to_le_bytes()); // block_count
        let pc = r10f::crc32_ieee(payload);
        hdr[0x50..0x54].copy_from_slice(&pc.to_le_bytes()); // raw_crc32
        hdr[0x54..0x58].copy_from_slice(&pc.to_le_bytes()); // payload_crc32
        // header_crc32：自身 4 字节先清 0 再算
        let hc = r10f::crc32_ieee(&hdr);
        hdr[0x58..0x5C].copy_from_slice(&hc.to_le_bytes());
        let mut content = hdr;
        content.extend_from_slice(payload);
        content.extend_from_slice(tail);
        content
    }

    /// 2 行 × 8px、stride 10 的 packed 载荷；每行取值不同，行错位会被断言抓到
    fn ramp_payload() -> Vec<u8> {
        let mut payload = Vec::new();
        for row in 0u16..2 {
            for g in 0u16..2 {
                let b = row * 8 + g * 4;
                payload.extend_from_slice(&pack_group([b, b + 1, 1022 - b, 1023 - b]));
            }
        }
        payload
    }

    #[test]
    fn crc32_known_vector() {
        assert_eq!(r10f::crc32_ieee(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn raw10_high8_length_and_values() {
        let packed = ramp_payload();
        assert_eq!(packed.len(), 20);
        let out = raw10_to_gray8(&packed, 8, 2, 10).expect("解包应成功");
        assert_eq!(out.len(), 8 * 2, "输出像素数必须是 width*height，不含行填充");
        let want: Vec<u8> = (0u16..2)
            .flat_map(|row| {
                (0u16..2).flat_map(move |g| {
                    let b = row * 8 + g * 4;
                    [
                        (b >> 2) as u8,
                        ((b + 1) >> 2) as u8,
                        ((1022 - b) >> 2) as u8,
                        ((1023 - b) >> 2) as u8,
                    ]
                })
            })
            .collect();
        assert_eq!(out, want);
        // 10bit 0x2AB(683) 的高 8 位 = 0xAA；低 2 位不会漏进预览
        assert_eq!(pack_group([683, 0, 1023, 1])[0], 0xAA);
        assert_eq!(683u16 >> 2, 0xAA);
    }

    #[test]
    fn raw10_high8_skips_row_padding() {
        // 每行实际 10B、stride 12（2B 填充 0xEE）：填充不得被当成像素
        let mut packed = Vec::new();
        for row in 0u16..2 {
            let v = row * 100;
            for _ in 0..2 {
                packed.extend_from_slice(&pack_group([v, v, v, v]));
            }
            packed.extend_from_slice(&[0xEE, 0xEE]);
        }
        let out = raw10_to_gray8(&packed, 8, 2, 12).expect("解包应成功");
        let mut want = vec![0u8; 8];
        want.extend_from_slice(&[25u8; 8]); // 100 >> 2 == 25
        assert_eq!(out, want);
    }

    #[test]
    fn raw10_high8_rejects_bad_geometry() {
        let packed = vec![0u8; 20];
        assert!(raw10_to_gray8(&packed, 8, 2, 10).is_ok());
        assert!(raw10_to_gray8(&packed, 6, 2, 8).is_err(), "width 非 4 的倍数");
        assert!(raw10_to_gray8(&packed, 8, 2, 9).is_err(), "stride 装不下一行");
        assert!(raw10_to_gray8(&packed[..19], 8, 2, 10).is_err(), "载荷被截断");
        assert!(raw10_to_gray8(&packed, 8, 3, 10).is_err(), "行数不够");
        assert!(raw10_to_gray8(&packed, 0, 2, 10).is_err(), "零宽");
    }

    #[test]
    fn r10f_preview_end_to_end() {
        let payload = ramp_payload();
        let tail_len = r10f::dma_bytes(payload.len() as u32) as usize
            - r10f::HDR_BYTES
            - payload.len();
        let content = synthetic_r10f(&payload, 8, 2, 10, &vec![0u8; tail_len]);
        assert_eq!(content.len(), r10f::dma_bytes(payload.len() as u32) as usize);

        let (h, preview) = r10f_gray8(&content, 8, 2)
            .expect("R10F 应通过严格校验")
            .expect("应识别为 R10F");
        assert_eq!((h.width, h.height, h.stride), (8, 2, 10));
        assert_eq!(preview.len(), 16);
        assert_eq!(preview, raw10_to_gray8(&payload, 8, 2, 10).unwrap());

        // 已授权几何与头里不一致 → 拒绝
        assert!(r10f_gray8(&content, 16, 2).is_err());
        // 载荷改一个字节 → payload CRC 不过 → 拒绝
        let mut bad = content.clone();
        bad[r10f::HDR_BYTES] ^= 0xff;
        assert!(r10f_gray8(&bad, 8, 2).is_err());
        // 头截断（payload_bytes 超出槽内容）→ 拒绝
        assert!(r10f_gray8(&content[..140], 8, 2).is_err());
        // 只截掉尾部 4KiB 对齐零（Python 严格实现也拒）→ 拒绝
        let no_tail = &content[..r10f::HDR_BYTES + payload.len()];
        assert!(r10f_gray8(no_tail, 8, 2).is_err(), "缺 transport tail 必须拒绝");
        // 旧 8bit 槽内容 → Ok(None)，不误判成 R10F
        let eight = vec![0x5Au8; 64];
        assert!(r10f_gray8(&eight, 8, 2).expect("不是 R10F").is_none());
    }

    /// 1080p/4K 的槽长与槽步长必须和驱动推导一致；4K 单路两槽 + mailbox
    /// 的分配量 = 20,754,432B。槽长写错（例如 10,370,112）必须被拒绝。
    #[test]
    fn descriptor_check_1080p_and_4k() {
        let mut info = IocFrame::default();
        info.phys_addr = 0x1000_0000;
        info.hdr_bytes = 128;
        info.slots = 2;

        info.width = 1920;
        info.height = 1080;
        info.slot_bytes = 2_592_768;
        info.stride = 2_596_864;
        info.mb_off = 5_193_728;
        info.size = 5_197_824;
        assert!(descriptor_check(&info), "1080p 布局必须通过");

        info.width = 3840;
        info.height = 2160;
        info.slot_bytes = 10_371_072;
        info.stride = 10_375_168;
        info.mb_off = 20_750_336;
        info.size = 20_754_432;
        assert!(descriptor_check(&info), "4K 布局必须通过");

        info.slot_bytes = 10_370_112;
        assert!(!descriptor_check(&info), "非 4KiB 对齐的槽长必须拒绝");
    }
}
