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
    FrameMailbox, FrameSlotHdr, IocFrame, IocFrameWait, ANLOGIC_FRAME_FLAG_COMPLETE,
    ANLOGIC_FRAME_MAGIC, ANLOGIC_FRAME_MAX_WAIT_MS, ANLOGIC_IOCFRAMEINFO, ANLOGIC_IOCFRAMEWAIT,
};
use anlogic_pci_tools::parse::strtoul_base0;
use anlogic_pci_tools::rka1::r10f;
use anlogic_pci_tools::sys::{self, Mmap, PROT_READ};

const DEF_DEV: &str = "/dev/ANLOGIC-PCI0_frame";
const DEF_OUT: &str = "dma_frame.pgm";
const DEF_TIMEOUT_MS: u32 = 3000;

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
        "用法：{} [-d 设备节点] [-t 超时ms] [-n] [-o 输出.pgm] [--info] [输出.pgm]\n\
         \x20 -d  收帧缓冲节点，默认 {}\n\
         \x20 -t  等帧超时（毫秒，1..600000），默认 {}\n\
         \x20 -n  只在出现更新的帧时返回（默认：缓冲里已有帧就直接取最新一帧）\n\
         \x20 -o  输出 PGM 文件名，默认 {}\n\
         \x20 --info 只打印缓冲物理地址/几何/序号，不取帧\n\
         退出码：0 成功，2 设备不可用，3 描述符不符，4 无新帧，5 一致性不过，6 输出错误\n",
        prog, DEF_DEV, DEF_TIMEOUT_MS, DEF_OUT
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
/// 字节序与 `app/reference/stereo/parallel/frame/frame_contract.py::unpack_raw10` /
/// `tools/pcie_integration/r10f_strict.py` 一致：每组 `P0[9:2] P1[9:2] P2[9:2] P3[9:2]
/// {P3[1:0],P2[1:0],P1[1:0],P0[1:0]}`。预览取每个 10bit 采样的**高 8 位**（`>> 2`，
/// 截断不四舍五入；也就是每组前 4 个字节）。
///
/// `stride` 是 R10F 头里的**行字节数**（1920 宽时 2400）；行尾填充不会进图。
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
    let h = r10f::parse(hdr, payload, tail, expect_w as u16, expect_h as u16, None)?;
    let preview = raw10_to_gray8(payload, h.width as usize, h.height as usize, h.stride as usize)?;
    Ok(Some((h, preview)))
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
        LongOpt { name: "help", has_arg: false, val: b'h' as u32 },
    ];

    let mut getopt = Getopt::new(&argv, "d:t:no:h", &longopts);

    let mut dev_path = DEF_DEV.to_string();
    let mut out_path: Option<String> = None;
    let mut timeout_ms: u32 = DEF_TIMEOUT_MS;
    let mut wait_new = false;
    let mut want_info = false;

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
     * 1920x1080 时是 2,592,768 字节，而 width*height 只有 2,073,600。
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
    //   判据与 `tools/pcie_integration/r10f_strict.py` / contract/acceptance-vectors.json
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
    use super::{r10f_gray8, raw10_to_gray8};
    use anlogic_pci_tools::rka1::r10f;

    /// 4 个 10bit 样本 → 5 字节（与
    /// `app/reference/stereo/parallel/frame/frame_contract.py::pack_raw10` 同字节序）
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
}
