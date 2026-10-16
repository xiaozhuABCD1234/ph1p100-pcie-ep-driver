// SPDX-License-Identifier: GPL-2.0
/*!
 * link_ctl —— 相机链路 / 推帧的查看与手动修补
 *
 * 驱动在探测时已经把「下 bit / 重启之后必须做的初始化」做完了（写 CTRL 使能位、
 * 填推帧目标与门铃地址、按需重触发传感器），这个工具是那三步的用户态出口：
 *   status  查链路与推帧状态（链路是否锁、DDR3 是否就绪、已推帧数、最近错误码、
 *           推帧目标/门铃地址的现值）；
 *   retrig  手动重新触发一次相机初始化（链路掉了之后恢复用）；
 *   push    复核推帧目标与门铃地址；给了地址就是重设。
 *
 * 三条命令对应三条 ioctl（编号与结构体在 include/anlogic_pci_ioctl.h，本文件只按
 * 那份 ABI 声明一遍）。驱动侧任何一次寄存器读都先过 BAR0+0x40 的 MAGIC 门禁、
 * 读回 0xffffffff 立即熔断，本工具因此不需要自己再抄一套校验；它只用 ioctl，
 * 不 mmap BAR、也不碰任何寄存器（要单笔读写寄存器用 pci_bar）。
 *
 * 用法：
 *   link_ctl [选项] [status|retrig|push]
 *
 * 选项：
 *   -d <节点>      控制节点，默认 /dev/ANLOGIC-PCI0_control
 *   -t <毫秒>      retrig 触发后最多等多久等链路锁定（默认 2000，0 = 不等，上限 5000）
 *   --dst-lo <v>   推帧目标地址低 32 位（与 --dst-hi 一起给）
 *   --dst-hi <v>   推帧目标地址高 32 位
 *   --db-lo <v>    门铃地址低 32 位（与 --db-hi 一起给）
 *   --db-hi <v>    门铃地址高 32 位
 *   -q             少打几行
 *   -h             用法
 *
 * 退出码：
 *   0 正常（链路已锁、推帧已初始化、最近一次推帧无错、DDR3 就绪）
 *   1 命令行用法错误
 *   2 设备不可用（open/ioctl 失败）
 *   3 相机链路未锁（lane_valid==0）
 *   4 推帧未初始化（CTRL bit6 没置位）
 *   5 推帧最近一次报错（0xC8 != 0）或 DDR3 未就绪
 */
use std::ffi::c_void;
use std::os::fd::{AsRawFd, RawFd};

use anlogic_pci_tools::getopt::{Getopt, LongOpt};
use anlogic_pci_tools::ioctl::{
    IocLinkRetrig, IocLinkStatus, IocPushCfg, IocPushCfgB, ANLOGIC_CAM_LANE_ALL,
    ANLOGIC_CTRL_DDR_EN, ANLOGIC_CTRL_ENABLE, ANLOGIC_CTRL_MCLK_SEL, ANLOGIC_CTRL_PUSH_EN,
    ANLOGIC_CTRL_PUSH_EN_B, ANLOGIC_CTRL_PUSH_SRC, ANLOGIC_DDR_STATUS_ALIVE,
    ANLOGIC_DDR_STATUS_CAL_DONE, ANLOGIC_IOCLINKRETRIG, ANLOGIC_IOCLINKSTATUS,
    ANLOGIC_IOCPUSHCFG, ANLOGIC_IOCPUSHCFGB, ANLOGIC_PUSH_CFG_SET_DB,
    ANLOGIC_PUSH_CFG_SET_DST, ANLOGIC_PUSH_ERR_NAMES, ANLOGIC_PUSH_STATUS_BUSY,
    ANLOGIC_PUSH_STATUS_DB_SENT, ANLOGIC_PUSH_STATUS_LAST_OK,
};
use anlogic_pci_tools::parse::strtoul_base0;
use anlogic_pci_tools::sys;

const DEF_DEV: &str = "/dev/ANLOGIC-PCI0_control";
const DEF_WAIT_MS: u32 = 2000;
const MAX_WAIT_MS: u32 = 5000;

const EX_OK: i32 = 0;
const EX_USAGE: i32 = 1;
const EX_DEV: i32 = 2;
const EX_NO_LOCK: i32 = 3;
const EX_NOT_ARMED: i32 = 4;
const EX_PUSH_ERR: i32 = 5;

const EXIT_HELP: &str =
    "退出码：0 正常，1 用法，2 设备不可用，3 链路未锁，4 推帧未初始化，5 推帧报错/DDR 未就绪";

/// 一条 64 位地址按「低 32 位在前」拼回来（寄存器里就是 lo/hi 两个字）
fn addr64(lo: u32, hi: u32) -> u64 {
    ((hi as u64) << 32) | lo as u64
}

/// 取一个位（用于把寄存器原始值解码成 0/1 打印）
fn bit(v: u32, mask: u32) -> u32 {
    if v & mask != 0 {
        1
    } else {
        0
    }
}

fn num32(s: &str, what: &str) -> Result<u32, String> {
    match strtoul_base0(s) {
        Some(v) if v <= u32::MAX as u64 => Ok(v as u32),
        _ => Err(format!("{} 需要 0..0xffffffff 的数值（收到 '{}'）", what, s)),
    }
}

fn link_status(fd: RawFd) -> Result<IocLinkStatus, std::io::Error> {
    let mut st = IocLinkStatus::default();
    // SAFETY: st 就是 ANLOGIC_IOCLINKSTATUS 对应的结构体
    unsafe {
        sys::ioctl_ptr(
            fd,
            ANLOGIC_IOCLINKSTATUS,
            &mut st as *mut IocLinkStatus as *mut c_void,
        )
    }?;
    Ok(st)
}

fn print_status(st: &IocLinkStatus, quiet: bool) {
    println!(
        "链路    : 相机 A 路 {}（lane_valid=0x{:x}，四条全对齐 = 0x{:x}）  状态字 0x{:08x}  0x4C 帧计数 {}",
        if st.lane_valid == ANLOGIC_CAM_LANE_ALL {
            "已锁"
        } else if st.lane_valid != 0 {
            "部分 lane 对齐"
        } else {
            "未锁"
        },
        st.lane_valid,
        ANLOGIC_CAM_LANE_ALL,
        st.cam_stat,
        st.frame_cnt
    );
    println!(
        "DDR3    : {}（0x{:08x}：valid={} busy={} err={} alive={} cal_done={}）  0xA0 已落 {} 帧，基址 0x{:x}，{}/帧",
        if st.ddr_ready != 0 { "就绪" } else { "未就绪" },
        st.ddr_status,
        bit(st.ddr_status, 1),
        bit(st.ddr_status, 2),
        bit(st.ddr_status, 4),
        bit(st.ddr_status, ANLOGIC_DDR_STATUS_ALIVE),
        bit(st.ddr_status, ANLOGIC_DDR_STATUS_CAL_DONE),
        st.ddr_seq,
        st.ddr_base,
        st.ddr_len
    );
    println!(
        "推帧    : {}  0xC0 已推 {} 帧，最近一帧 {} 笔 MWr，0xC4=0x{:08x}（busy={} last_ok={} 门铃已发={}）",
        if st.push_en != 0 { "已初始化（CTRL bit6=1）" } else { "**未初始化**（CTRL bit6=0）" },
        st.pushed,
        st.push_tlp_cnt,
        st.push_status,
        bit(st.push_status, ANLOGIC_PUSH_STATUS_BUSY),
        bit(st.push_status, ANLOGIC_PUSH_STATUS_LAST_OK),
        bit(st.push_status, ANLOGIC_PUSH_STATUS_DB_SENT)
    );
    println!(
        "推帧错误: {}（0xC8=0x{:08x} = {}）",
        if st.push_err == 0 { "无" } else { "**有**" },
        st.push_err,
        ANLOGIC_PUSH_ERR_NAMES
            .get(st.push_err as usize)
            .copied()
            .unwrap_or("未知错误码")
    );
    println!(
        "推帧源  : {}",
        if st.push_src != 0 {
            "片内 ERAM（绕开 DDR3）"
        } else {
            "DDR3 里已发布的那一帧"
        }
    );
    println!("收帧缓冲: mailbox seq={} count={}（驱动按门铃补齐）", st.seq, st.count);
    if !quiet {
        println!(
            "CTRL    : 0x{:08x}（使能={} 触发={} 重触发电平={} DDR搬运={} 推帧A={} 推帧源={} 推帧B={}）",
            st.ctrl,
            bit(st.ctrl, ANLOGIC_CTRL_ENABLE),
            bit(st.ctrl, 2),
            bit(st.ctrl, ANLOGIC_CTRL_MCLK_SEL),
            bit(st.ctrl, ANLOGIC_CTRL_DDR_EN),
            bit(st.ctrl, ANLOGIC_CTRL_PUSH_EN),
            bit(st.ctrl, ANLOGIC_CTRL_PUSH_SRC),
            bit(st.ctrl, ANLOGIC_CTRL_PUSH_EN_B)
        );
    }
    println!(
        "推帧目标: 0x{:016x}（0xB0/0xB4 = 0x{:08x}/0x{:08x}）",
        addr64(st.dst_lo, st.dst_hi),
        st.dst_lo,
        st.dst_hi
    );
    println!(
        "门铃地址: 0x{:016x}（0xB8/0xBC = 0x{:08x}/0x{:08x}）{}",
        addr64(st.db_lo, st.db_hi),
        st.db_lo,
        st.db_hi,
        if st.armed != 0 {
            "（驱动探测时初始化过）"
        } else {
            "（驱动没初始化成——看 dmesg 里驱动的报错）"
        }
    );
}

fn push_cfg(
    fd: RawFd,
    set_dst: Option<(u32, u32)>,
    set_db: Option<(u32, u32)>,
) -> Result<IocPushCfg, std::io::Error> {
    let mut cfg = IocPushCfg::default();
    if let Some((lo, hi)) = set_dst {
        cfg.flags |= ANLOGIC_PUSH_CFG_SET_DST;
        cfg.dst_lo = lo;
        cfg.dst_hi = hi;
    }
    if let Some((lo, hi)) = set_db {
        cfg.flags |= ANLOGIC_PUSH_CFG_SET_DB;
        cfg.db_lo = lo;
        cfg.db_hi = hi;
    }
    // SAFETY: cfg 就是 ANLOGIC_IOCPUSHCFG 对应的结构体
    unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCPUSHCFG, &mut cfg as *mut IocPushCfg as *mut c_void) }?;
    Ok(cfg)
}

/// B 路推帧目标/门铃（0x100~0x10C）：与 A 路同一套语义，只是另一组寄存器
fn pushb_cfg(
    fd: RawFd,
    set_dst: Option<(u32, u32)>,
    set_db: Option<(u32, u32)>,
) -> Result<IocPushCfgB, std::io::Error> {
    let mut cfg = IocPushCfgB::default();
    if let Some((lo, hi)) = set_dst {
        cfg.flags |= ANLOGIC_PUSH_CFG_SET_DST;
        cfg.dst_lo = lo;
        cfg.dst_hi = hi;
    }
    if let Some((lo, hi)) = set_db {
        cfg.flags |= ANLOGIC_PUSH_CFG_SET_DB;
        cfg.db_lo = lo;
        cfg.db_hi = hi;
    }
    // SAFETY: cfg 就是 ANLOGIC_IOCPUSHCFGB 对应的结构体
    unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCPUSHCFGB, &mut cfg as *mut IocPushCfgB as *mut c_void) }?;
    Ok(cfg)
}

fn print_push_cfg_b(cfg: &IocPushCfgB) {
    println!(
        "B 路推帧目标: 0x{:016x}（0x100/0x104 = 0x{:08x}/0x{:08x}）",
        addr64(cfg.dst_lo, cfg.dst_hi),
        cfg.dst_lo,
        cfg.dst_hi
    );
    println!(
        "B 路门铃地址: 0x{:016x}（0x108/0x10C = 0x{:08x}/0x{:08x}）",
        addr64(cfg.db_lo, cfg.db_hi),
        cfg.db_lo,
        cfg.db_hi
    );
    println!(
        "本次写入: {}（applied=0x{:x}）  B 路推帧使能={}  驱动初始化过={}",
        if cfg.applied == 0 { "无（只复核）" } else { "已写并回读复核通过" },
        cfg.applied,
        cfg.push_en,
        cfg.armed
    );
    if cfg.applied != 0 && cfg.armed == 0 {
        eprintln!("!! 注意：地址写进去了，但驱动记的初始化状态是「没成」——看 dmesg 里驱动报的错。");
    }
}

fn print_push_cfg(cfg: &IocPushCfg) {
    println!(
        "推帧目标: 0x{:016x}（0xB0/0xB4 = 0x{:08x}/0x{:08x}）",
        addr64(cfg.dst_lo, cfg.dst_hi),
        cfg.dst_lo,
        cfg.dst_hi
    );
    println!(
        "门铃地址: 0x{:016x}（0xB8/0xBC = 0x{:08x}/0x{:08x}）",
        addr64(cfg.db_lo, cfg.db_hi),
        cfg.db_lo,
        cfg.db_hi
    );
    println!(
        "本次写入: {}（applied=0x{:x}）  推帧使能={}  推帧源={}  驱动初始化过={}",
        if cfg.applied == 0 { "无（只复核）" } else { "已写并回读复核通过" },
        cfg.applied,
        cfg.push_en,
        cfg.push_src,
        cfg.armed
    );
    if cfg.applied != 0 && cfg.armed == 0 {
        eprintln!("!! 注意：地址写进去了，但驱动记的初始化状态是「没成」——看 dmesg 里驱动报的错。");
    }
}

fn usage(prog: &str) {
    eprint!(
        "用法: {} [选项] [status|retrig|push|pushb]\n\
         \n\
         查/改 PH1P100 的相机链路与推帧通路（都走驱动 ioctl，本工具不 mmap BAR）。\n\
         \x20 status  查链路与推帧状态（默认）\n\
         \x20 retrig  手动重新触发一次相机初始化（链路掉了之后恢复用）\n\
         \x20 push    复核 A 路推帧目标/门铃地址；给了 --dst-*/--db-* 就是重设\n\
         \x20 pushb   同上，但打 B 路那组寄存器（0x100~0x10C）\n\
         \n\
         选项:\n\
         \x20 -d <节点>     控制节点，默认 {}\n\
         \x20 -t <毫秒>     retrig 后最多等多久等锁定，默认 {}（0..{}，0 = 不等）\n\
         \x20 --dst-lo <v>  推帧目标地址低 32 位\n\
         \x20 --dst-hi <v>  推帧目标地址高 32 位\n\
         \x20 --db-lo <v>   门铃地址低 32 位\n\
         \x20 --db-hi <v>   门铃地址高 32 位\n\
         \x20 -q            少打几行\n\
         \x20 -h            用法\n\
         \n\
         {}\n",
        prog, DEF_DEV, DEF_WAIT_MS, MAX_WAIT_MS, EXIT_HELP
    );
}

fn run() -> i32 {
    let argv: Vec<String> = std::env::args_os()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();

    let longopts = [
        LongOpt { name: "dev", has_arg: true, val: b'd' as u32 },
        LongOpt { name: "timeout", has_arg: true, val: b't' as u32 },
        LongOpt { name: "dst-lo", has_arg: true, val: 2 },
        LongOpt { name: "dst-hi", has_arg: true, val: 3 },
        LongOpt { name: "db-lo", has_arg: true, val: 4 },
        LongOpt { name: "db-hi", has_arg: true, val: 5 },
        LongOpt { name: "quiet", has_arg: false, val: b'q' as u32 },
        LongOpt { name: "help", has_arg: false, val: b'h' as u32 },
    ];

    let mut getopt = Getopt::new(&argv, "d:t:qh", &longopts);

    let mut dev_path = DEF_DEV.to_string();
    let mut wait_ms = DEF_WAIT_MS;
    let mut quiet = false;
    let mut dst_lo: Option<u32> = None;
    let mut dst_hi: Option<u32> = None;
    let mut db_lo: Option<u32> = None;
    let mut db_hi: Option<u32> = None;

    while let Some(item) = getopt.next() {
        let (id, arg) = match item {
            Ok(o) => o,
            Err(msg) => {
                eprintln!("{}: {}", argv[0], msg);
                usage(&argv[0]);
                return EX_USAGE;
            }
        };
        let r: Result<(), String> = match id as u8 {
            b'd' => {
                dev_path = arg.unwrap_or_default();
                Ok(())
            }
            b't' => {
                let s = arg.unwrap_or_default();
                match strtoul_base0(&s) {
                    Some(v) if v <= MAX_WAIT_MS as u64 => {
                        wait_ms = v as u32;
                        Ok(())
                    }
                    _ => Err(format!("--timeout 需要 0..{} 的毫秒数（收到 '{}'）", MAX_WAIT_MS, s)),
                }
            }
            b'q' => {
                quiet = true;
                Ok(())
            }
            b'h' => {
                usage(&argv[0]);
                return EX_OK;
            }
            2 | 3 | 4 | 5 => {
                let (slot, what) = match id {
                    2 => (&mut dst_lo, "--dst-lo"),
                    3 => (&mut dst_hi, "--dst-hi"),
                    4 => (&mut db_lo, "--db-lo"),
                    _ => (&mut db_hi, "--db-hi"),
                };
                let s = arg.unwrap_or_default();
                match num32(&s, what) {
                    Ok(v) => {
                        *slot = Some(v);
                        Ok(())
                    }
                    Err(m) => Err(m),
                }
            }
            _ => Err("未知选项".to_string()),
        };
        if let Err(msg) = r {
            eprintln!("!! {}", msg);
            usage(&argv[0]);
            return EX_USAGE;
        }
    }

    let positional: Vec<String> = getopt.positional();
    let cmd = positional.first().map(|s| s.as_str()).unwrap_or("status");
    if positional.len() > 1 {
        eprintln!("!! 多余的参数：'{}'", positional[1]);
        usage(&argv[0]);
        return EX_USAGE;
    }
    if cmd != "status" && cmd != "retrig" && cmd != "push" && cmd != "pushb" {
        eprintln!("!! 不认识的动作 '{}'（只有 status / retrig / push）", cmd);
        usage(&argv[0]);
        return EX_USAGE;
    }
    // 地址的两个半字必须成对给全，免得高 32 位被悄悄当成 0
    if dst_lo.is_some() != dst_hi.is_some() {
        eprintln!("!! 推帧目标地址要 lo/hi 两个半字一起给（--dst-lo 与 --dst-hi）");
        return EX_USAGE;
    }
    if db_lo.is_some() != db_hi.is_some() {
        eprintln!("!! 门铃地址要 lo/hi 两个半字一起给（--db-lo 与 --db-hi）");
        return EX_USAGE;
    }

    // ---- 打开控制节点（只读；三条 ioctl 的寄存器校验全部由驱动做）----
    let file = match std::fs::OpenOptions::new().read(true).open(&dev_path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("!! 打不开 {}：{}", dev_path, sys::errno_str(&e));
            eprintln!("   （需要 root；驱动已 insmod 且设备已枚举；节点名形如 /dev/ANLOGIC-PCI0_control）");
            return EX_DEV;
        }
    };
    let fd = file.as_raw_fd();

    if cmd == "retrig" {
        let mut rt = IocLinkRetrig::default();
        rt.wait_ms = wait_ms;
        println!("=== 手动重新触发相机初始化（最多等 {} ms）===", wait_ms);
        // SAFETY: rt 就是 ANLOGIC_IOCLINKRETRIG 对应的结构体
        if let Err(e) = unsafe {
            sys::ioctl_ptr(
                fd,
                ANLOGIC_IOCLINKRETRIG,
                &mut rt as *mut IocLinkRetrig as *mut c_void,
            )
        } {
            eprintln!("!! ANLOGIC_IOCLINKRETRIG 失败：{}", sys::errno_str(&e));
            eprintln!("   （-EIO：BAR0+0x40 MAGIC 不对或读到全 F；-EINVAL：wait_ms 超上限）");
            return EX_DEV;
        }
        println!(
            "  触发后 CTRL=0x{:08x}（重触发电平 bit2={}）、等了 {} ms",
            rt.ctrl,
            bit(rt.ctrl, ANLOGIC_CTRL_MCLK_SEL),
            rt.waited_ms
        );
        println!(
            "  A 路状态字 0x{:08x}，lane_valid=0x{:x}（四条全对齐 = 0x{:x}）、0x4C 帧计数 {}{}",
            rt.cam_stat,
            rt.lane_valid,
            ANLOGIC_CAM_LANE_ALL,
            rt.frame_cnt,
            if rt.frames_flowing != 0 { "（相对开始等时变过 ⇒ 帧已经在出）" } else { "（没动）" }
        );
        if rt.lane_valid == ANLOGIC_CAM_LANE_ALL {
            println!("  ⇒ 四条 lane 全部对齐，链路已锁");
        } else if rt.frames_flowing != 0 {
            println!(
                "  ⇒ 已经在出帧（链路功能上通了），但 lane_valid 还没到 0x{:x}：\n     \
                 本板实测重触发后 lane 是逐步对齐的（先 0x8、中间可能掉回 0、几秒后 0xf），再等几秒用 status 复核",
                ANLOGIC_CAM_LANE_ALL
            );
        } else {
            eprintln!("!! 等完既没全对齐也没出帧：MIPI 链路没起来（PHY/接线/供电/参考时钟），不是主机侧能补的");
            return EX_NO_LOCK;
        }
        // 重触发之后顺手把状态全打一遍，省得再跑一次
        println!();
        match link_status(fd) {
            Ok(st) => {
                print_status(&st, quiet);
                if st.push_en == 0 || st.armed == 0 {
                    return EX_NOT_ARMED;
                }
                return EX_OK;
            }
            Err(e) => {
                eprintln!("!! ANLOGIC_IOCLINKSTATUS 失败：{}", sys::errno_str(&e));
                return EX_DEV;
            }
        }
    }

    if cmd == "push" {
        let dst = match (dst_lo, dst_hi) {
            (Some(lo), Some(hi)) => Some((lo, hi)),
            _ => None,
        };
        let db = match (db_lo, db_hi) {
            (Some(lo), Some(hi)) => Some((lo, hi)),
            _ => None,
        };
        println!("=== 推帧目标 / 门铃地址 ===");
        return match push_cfg(fd, dst, db) {
            Ok(cfg) => {
                print_push_cfg(&cfg);
                EX_OK
            }
            Err(e) => {
                eprintln!("!! ANLOGIC_IOCPUSHCFG 失败：{}", sys::errno_str(&e));
                eprintln!("   （-EIO：写进去读回不一致 / 读到全 F；-EINVAL：flags 有未知位）");
                EX_DEV
            }
        };
    }

    if cmd == "pushb" {
        let dst = match (dst_lo, dst_hi) {
            (Some(lo), Some(hi)) => Some((lo, hi)),
            _ => None,
        };
        let db = match (db_lo, db_hi) {
            (Some(lo), Some(hi)) => Some((lo, hi)),
            _ => None,
        };
        println!("=== B 路推帧目标 / 门铃地址 ===");
        return match pushb_cfg(fd, dst, db) {
            Ok(cfg) => {
                print_push_cfg_b(&cfg);
                EX_OK
            }
            Err(e) => {
                eprintln!("!! ANLOGIC_IOCPUSHCFGB 失败：{}", sys::errno_str(&e));
                eprintln!("   （-EIO：写进去读回不一致 / 读到全 F（老映像没有 0x100~0x10C 这组寄存器））");
                EX_DEV
            }
        };
    }

    // ---- status ----
    println!("=== 相机链路 / 推帧状态 ===");
    let st = match link_status(fd) {
        Ok(st) => st,
        Err(e) => {
            eprintln!("!! ANLOGIC_IOCLINKSTATUS 失败：{}", sys::errno_str(&e));
            eprintln!("   （-EIO：BAR0+0x40 MAGIC 不对或读到全 F，设备可能已经掉了）");
            return EX_DEV;
        }
    };
    print_status(&st, quiet);

    if st.cam_locked == 0 {
        eprintln!("!! 相机链路没锁（lane_valid=0）：可用 `link_ctl retrig` 重新初始化传感器");
        return EX_NO_LOCK;
    }
    if st.lane_valid != ANLOGIC_CAM_LANE_ALL {
        eprintln!(
            "!! 只有 0x{:x}/0x{:x} 条 lane 对齐（状态字 0x{:08x}）：链路能出帧但不完整，\n   \
             可用 `link_ctl retrig` 重跑一次传感器初始化；若帧计数在涨，说明数据已经在出",
            st.lane_valid, ANLOGIC_CAM_LANE_ALL, st.cam_stat
        );
        return EX_NO_LOCK;
    }
    if st.push_en == 0 || st.armed == 0 {
        eprintln!(
            "!! 推帧没初始化（CTRL bit6={}、驱动初始化过={}）",
            bit(st.ctrl, ANLOGIC_CTRL_PUSH_EN),
            st.armed
        );
        return EX_NOT_ARMED;
    }
    if st.push_err != 0 || st.ddr_ready == 0 {
        eprintln!("!! DDR3 未就绪或推帧报错，看上面两行");
        return EX_PUSH_ERR;
    }
    EX_OK
}

fn main() {
    let code = run();
    anlogic_pci_tools::exit_with(code);
}
