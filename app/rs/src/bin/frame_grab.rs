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
use std::ffi::c_void;
use std::io::Write;
use std::os::fd::{AsRawFd, RawFd};

use anlogic_pci_tools::getopt::{Getopt, LongOpt};
use anlogic_pci_tools::ioctl::{
    FrameMailbox, FrameSlotHdr, IocFrame, IocFrameWait, ANLOGIC_FRAME_FLAG_COMPLETE,
    ANLOGIC_FRAME_MAGIC, ANLOGIC_FRAME_MAX_WAIT_MS, ANLOGIC_IOCFRAMEINFO, ANLOGIC_IOCFRAMEWAIT,
};
use anlogic_pci_tools::parse::strtoul_base0;
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
    if info.size < info.hdr_bytes as u64 + info.stride as u64 * info.slots as u64 {
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
        "  槽位                        : {} 槽，步长 {} 字节，槽头 {} 字节（槽 n 在 phys + {}×n）",
        info.slots, info.stride, info.hdr_bytes, info.stride
    );
    println!(
        "  mailbox                     : 缓冲内偏移 {}（独占一页，FPGA 把 seq/count 写这里）",
        info.mb_off
    );
    println!(
        "  每帧                        : {}x{} 灰度 = {} 字节",
        info.width, info.height, info.slot_bytes
    );
    println!(
        "  序号/计数（FPGA 写）        : seq={} count={}{}",
        info.seq,
        info.count,
        if info.seq != 0 { "" } else { "（seq 还是 0：FPGA 还没往这块缓冲写过帧）" }
    );
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

    // ---- 6. 搬像素（不整帧就补 0 并告警）----
    let npix = info.width as usize * info.height as usize;
    let mut img = vec![0u8; npix];
    let mut copy = hdr_len as usize;
    if copy > info.slot_bytes as usize {
        eprintln!("!! 槽头 len={} 超过一帧 {} 字节，按一帧夹紧", hdr_len, info.slot_bytes);
        copy = info.slot_bytes as usize;
        rc = EX_INCONSISTENT;
    }
    if copy > npix {
        eprintln!(
            "!! 槽头 len={} 大于 {}x{} 需要的 {} 字节，按图大小夹紧",
            hdr_len, info.width, info.height, npix
        );
        copy = npix;
    }
    let src = map.as_ptr() as usize + slot as usize * info.stride as usize + info.hdr_bytes as usize;
    // SAFETY: 源地址在只读映射内（descriptor_check 已核对 stride/slots/mb_off 都合得上）
    unsafe {
        std::ptr::copy_nonoverlapping(src as *const u8, img.as_mut_ptr(), copy);
    }
    if copy < npix {
        eprintln!("!! 帧数据只有 {}/{} 字节，缺的用 0 补（图不完整）", copy, npix);
        rc = EX_INCONSISTENT;
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
        .and_then(|_| fp.write_all(&img))
        .and_then(|_| fp.flush());
    if let Err(e) = written {
        eprintln!("!! 写 {} 失败：{}", out_path, sys::errno_str(&e));
        return EX_OUTPUT;
    }

    println!("=== 结果 ===");
    println!("  输出文件  : {}（P5 灰度 {}x{}，{} 字节像素）", out_path, info.width, info.height, npix);
    println!(
        "  取自      : 物理 0x{:x} + 槽 {}（偏移 {}）+ 槽头 {}",
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
