// SPDX-License-Identifier: GPL-2.0
/*!
 * link_ctl —— 相机链路 / 推帧的查看与手动修补
 *
 * 驱动在探测时已经把「下 bit / 重启之后必须做的初始化」做完了（写 CTRL 使能位、
 * 填推帧目标与门铃地址、按需重触发传感器），这个工具是那几步的用户态出口：
 *   status  查链路与推帧状态（链路是否锁、DDR3 是否就绪、已推帧数、最近错误码、
 *           推帧目标/门铃地址的现值）；
 *   retrig  手动重新触发一次相机初始化（链路掉了之后恢复用）；
 *   push    复核推帧目标与门铃地址；给了地址就是重设；
 *   arm     选 A/B 路并申请一个 one-shot 额度（写 CTRL 前做 bit1/bit2 门禁与回读复核）；
 *   ack     匹配当前 HELD 的 push_done 并释放，然后把 CTRL 收回 0x21。
 *
 * 除 arm/ack 需要写 CTRL(0x44) 外，其余命令只读；寄存器访问仍走控制节点的
 * single-dw ioctl，不 mmap BAR。驱动侧每次读都先过 BAR0+0x40 的 MAGIC 门禁。
 *
 * 用法：
 *   link_ctl [选项] [status|retrig|push|arm|ack|sync [start|abort|status]]
 *
 * 选项：
 *   -d <节点>      控制节点，默认 /dev/ANLOGIC-PCI0_control
 *   -t <毫秒>      retrig 触发后最多等多久等链路锁定（默认 2000，0 = 不等，上限 5000）
 *   --dst-lo <v>   推帧目标地址低 32 位（与 --dst-hi 一起给；必须等于驱动分配的槽 0 数据起点）
 *   --dst-hi <v>   推帧目标地址高 32 位
 *   --db-lo <v>    门铃地址低 32 位（与 --db-hi 一起给；必须等于 mailbox.push_dbell）
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
    IocBar, IocLinkRetrig, IocLinkStatus, IocPushCfg, ANLOGIC_BAR_CTRL_OFF, ANLOGIC_CAM_LANE_ALL,
    ANLOGIC_CTRL_CAM_B, ANLOGIC_CTRL_DDR_EN, ANLOGIC_CTRL_ENABLE, ANLOGIC_CTRL_MCLK_SEL,
    ANLOGIC_CTRL_PUSH_EN, ANLOGIC_CTRL_PUSH_SRC, ANLOGIC_CTRL_TRIGGER, ANLOGIC_DDR_STATUS_ALIVE,
    ANLOGIC_DDR_STATUS_CAL_DONE, ANLOGIC_IOCLINKRETRIG, ANLOGIC_IOCLINKSTATUS, ANLOGIC_IOCPUSHCFG,
    ANLOGIC_IOCR, ANLOGIC_IOCW, ANLOGIC_PUSH_CFG_SET_DB, ANLOGIC_PUSH_CFG_SET_DST,
    ANLOGIC_PUSH_ERR_NAMES, ANLOGIC_PUSH_STATUS_BUSY, ANLOGIC_PUSH_STATUS_DB_SENT,
    ANLOGIC_PUSH_STATUS_LAST_OK,
};
use anlogic_pci_tools::parse::strtoul_base0;
use anlogic_pci_tools::rka1::Rka1;
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
    "退出码：0 正常，1 用法，2 设备不可用，3 链路未锁，4 推帧未初始化，5 推帧报错/DDR 未就绪或 arm/ack 失败";

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
        _ => Err(format!(
            "{} 需要 0..0xffffffff 的数值（收到 '{}'）",
            what, s
        )),
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

const CTRL_IDLE: u32 = ANLOGIC_CTRL_ENABLE | ANLOGIC_CTRL_DDR_EN;
const CTRL_WRITE_MASK: u32 = ANLOGIC_CTRL_ENABLE
    | ANLOGIC_CTRL_TRIGGER
    | ANLOGIC_CTRL_MCLK_SEL
    | ANLOGIC_CTRL_CAM_B
    | ANLOGIC_CTRL_DDR_EN
    | ANLOGIC_CTRL_PUSH_EN
    | ANLOGIC_CTRL_PUSH_SRC;

fn rka_err(e: (i32, String)) -> String {
    e.1
}

fn ctrl_read(fd: RawFd) -> Result<u32, String> {
    let mut bar = IocBar {
        bar_id: 0,
        bar_type: 0,
        bar_offaddr: ANLOGIC_BAR_CTRL_OFF,
        wdata: 0,
        rdata: 0,
    };
    // SAFETY: bar 就是 ANLOGIC_IOCR 对应的结构体
    unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCR, &mut bar as *mut IocBar as *mut c_void) }
        .map_err(|e| format!("ANLOGIC_IOCR(CTRL) 失败：{}", sys::errno_str(&e)))?;
    Ok(bar.rdata)
}

fn bar_read(fd: RawFd, off: u32) -> Result<u32, String> {
    let mut bar = IocBar {
        bar_id: 0,
        bar_type: 0,
        bar_offaddr: off,
        wdata: 0,
        rdata: 0,
    };
    // SAFETY: bar 就是 ANLOGIC_IOCR 对应的结构体
    unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCR, &mut bar as *mut IocBar as *mut c_void) }
        .map_err(|e| format!("ANLOGIC_IOCR(0x{:x}) 失败：{}", off, sys::errno_str(&e)))?;
    Ok(bar.rdata)
}

fn bar_write(fd: RawFd, off: u32, val: u32) -> Result<(), String> {
    let mut bar = IocBar {
        bar_id: 0,
        bar_type: 0,
        bar_offaddr: off,
        wdata: val,
        rdata: 0,
    };
    // SAFETY: bar 就是 ANLOGIC_IOCW 对应的结构体
    unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCW, &mut bar as *mut IocBar as *mut c_void) }
        .map_err(|e| format!("ANLOGIC_IOCW(0x{:x}) 失败：{}", off, sys::errno_str(&e)))?;
    Ok(())
}

fn sync_status(fd: RawFd) -> Result<(), String> {
    let st = bar_read(fd, 0x1e4)?;
    let skew = bar_read(fd, 0x1e8)? as i32;
    let pairs = bar_read(fd, 0x1ec)?;
    let warmup = bar_read(fd, 0x1f0)?;
    let timestamp = bar_read(fd, 0x1f4)?;
    let miss_a = bar_read(fd, 0x1f8)?;
    let miss_b = bar_read(fd, 0x1fc)?;
    let overrun = bar_read(fd, 0x200)?;
    let relock = bar_read(fd, 0x204)?;
    let seq_a = bar_read(fd, 0x208)?;
    let seq_b = bar_read(fd, 0x20c)?;
    println!("=== 硬件同步曝光状态 ===");
    println!(
        "  ready={} locked={} busy={} stopped={} pair_valid={} error={}",
        bit(st, 1 << 0),
        bit(st, 1 << 2),
        bit(st, 1 << 3),
        bit(st, 1 << 4),
        bit(st, 1 << 5),
        (st >> 8) & 0xf
    );
    println!(
        "  warmup={} pair_count={} skew={} tick={}",
        warmup, pairs, skew, timestamp
    );
    println!(
        "  pair_seq A/B={}/{} miss A/B={}/{} overrun A/B={}/{} relock={}",
        seq_a,
        seq_b,
        miss_a,
        miss_b,
        overrun & 0xffff,
        overrun >> 16,
        relock
    );
    Ok(())
}

fn sync_cmd(fd: RawFd, start: bool, wait_ms: u32) -> Result<(), String> {
    bar_write(fd, 0x1e0, if start { 1 } else { 2 })?;
    println!("=== SYNC {} ===", if start { "START" } else { "ABORT" });
    let deadline = std::time::Instant::now() + std::time::Duration::from_millis(wait_ms as u64);
    loop {
        let st = bar_read(fd, 0x1e4)?;
        let ready = bit(st, 1 << 0) != 0;
        let locked = bit(st, 1 << 2) != 0;
        let busy = bit(st, 1 << 3) != 0;
        if start && ready && locked {
            println!("  ⇒ pair 已锁定");
            return sync_status(fd);
        }
        if !start && !busy {
            println!("  ⇒ 已停止/清理");
            return sync_status(fd);
        }
        if std::time::Instant::now() >= deadline {
            println!("  超时：ready={} locked={} busy={}", ready, locked, busy);
            return sync_status(fd);
        }
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}

fn ctrl_write_raw(fd: RawFd, val: u32) -> Result<(), String> {
    let mut bar = IocBar {
        bar_id: 0,
        bar_type: 0,
        bar_offaddr: ANLOGIC_BAR_CTRL_OFF,
        wdata: val,
        rdata: 0,
    };
    // SAFETY: bar 就是 ANLOGIC_IOCW 对应的结构体
    unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCW, &mut bar as *mut IocBar as *mut c_void) }
        .map_err(|e| format!("ANLOGIC_IOCW(CTRL) 失败：{}", sys::errno_str(&e)))?;
    Ok(())
}

/// 写 CTRL 的唯一入口：bit1 必须为 0、bit2 电平不许变，写后整字回读复核。
fn safe_ctrl_set(fd: RawFd, target: u32) -> Result<u32, String> {
    let cur = ctrl_read(fd)?;
    if cur & ANLOGIC_CTRL_MCLK_SEL != 0 {
        return Err(format!(
            "CTRL=0x{:08x} 的 bit2=1（重触发已锁存）：不要再写 CTRL，先重下位流+重启 RK3588",
            cur
        ));
    }
    if cur & ANLOGIC_CTRL_TRIGGER != 0 {
        return Err(format!(
            "CTRL=0x{:08x} 的 bit1=1（单帧触发位）：拒绝继续",
            cur
        ));
    }
    ctrl_write_raw(fd, target)?;
    let rb = ctrl_read(fd)?;
    if (rb ^ target) & CTRL_WRITE_MASK != 0 {
        return Err(format!(
            "CTRL 回读不符：写 0x{:08x} 读到 0x{:08x}（mask 0x{:08x}）",
            target, rb, CTRL_WRITE_MASK
        ));
    }
    if (rb ^ cur) & ANLOGIC_CTRL_MCLK_SEL != 0 {
        return Err(format!(
            "CTRL[2] 在写入后变了（0x{:08x} -> 0x{:08x}）：立即停手",
            cur, rb
        ));
    }
    Ok(rb)
}

/// 收掉 probe / 上一次抓帧留下的 HELD；HELD 优先 ACK(0xC0)，失败再 CANCEL。
fn drain_held(rka: &Rka1) -> Result<(), String> {
    let st = rka.probe().map_err(rka_err)?;
    if st.held == 0 && st.busy == 0 && st.armed == 0 {
        return Ok(());
    }
    if st.held != 0 && st.push_done != 0 {
        let _ = rka.ack(st.push_done, 0);
    }
    let st2 = rka.probe().map_err(rka_err)?;
    if st2.held != 0 || st2.busy != 0 || st2.armed != 0 {
        rka.cancel().map_err(rka_err)?;
    }
    Ok(())
}

fn do_arm(fd: RawFd, route_b: bool) -> Result<(), String> {
    let rka = Rka1::new(fd);
    rka.probe().map_err(rka_err)?;
    drain_held(&rka)?;

    let target = CTRL_IDLE | ANLOGIC_CTRL_PUSH_EN | if route_b { ANLOGIC_CTRL_CAM_B } else { 0 };
    let rb = safe_ctrl_set(fd, target)?;
    println!(
        "CTRL <- 0x{:08x}（{} 路、push_en=1、bit2=0）",
        rb,
        if route_b { "B" } else { "A" }
    );

    // probe 早于相机首帧时 link_armed 会停在 0；这里只复核地址并让驱动
    // 重新置位，不改变任何寄存器值。
    push_cfg(fd, None, None).map_err(|e| format!("复核推帧地址失败：{}", e))?;

    let rq = match rka.arm() {
        Ok(rq) => rq,
        Err(e) => {
            let _ = safe_ctrl_set(fd, CTRL_IDLE);
            return Err(format!("ARM 失败：{}", e.1));
        }
    };
    println!(
        "ARM ok：TX_STATUS=0x{:08x} armed={} held={} 0xC0={} 0xC8=0x{:x}",
        rq.tx_status, rq.armed, rq.held, rq.push_done, rq.push_err
    );
    println!("下一步：frame_grab -n -o 帧.pgm 取帧，取完执行 link_ctl ack");
    Ok(())
}

fn do_ack(fd: RawFd) -> Result<(), String> {
    let rka = Rka1::new(fd);
    let st = rka.probe().map_err(rka_err)?;
    let mut released = false;
    if st.held != 0 {
        if st.push_done != 0 {
            match rka.ack(st.push_done, 0) {
                Ok(_) => released = true,
                Err(e) => {
                    eprintln!("!! ACK {} 失败：{}；改 CANCEL", st.push_done, e.1);
                    let _ = rka.cancel();
                }
            }
        } else {
            let _ = rka.cancel();
        }
    } else if st.busy != 0 || st.armed != 0 {
        let _ = rka.cancel();
    }
    let rb = safe_ctrl_set(fd, CTRL_IDLE)?;
    let after = rka.probe().map_err(rka_err)?;
    if after.held != 0 {
        return Err(format!(
            "ACK 后仍 HELD（TX_STATUS=0x{:08x}，0xC0={}，0xC8=0x{:x}）",
            after.tx_status, after.push_done, after.push_err
        ));
    }
    println!(
        "{}；CTRL <- 0x{:08x}；TX_STATUS=0x{:08x} held=0 0xC0={}",
        if released {
            format!("ACK {} ok", st.push_done)
        } else {
            "当前没有 HELD，无需 ACK".to_string()
        },
        rb,
        after.tx_status,
        after.push_done
    );
    Ok(())
}

fn print_status(st: &IocLinkStatus, quiet: bool) {
    let cam = if st.ctrl & ANLOGIC_CTRL_CAM_B != 0 {
        "B"
    } else {
        "A"
    };
    println!(
        "链路    : 相机 {} 路（CTRL[8]）{}（lane_valid=0x{:x}，四条全对齐 = 0x{:x}）  状态字 0x{:08x}  0x4C 帧计数 {}",
        cam,
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
    // TLP 数由帧长推导：RTL 每笔 MWr 128B。1080p = 20,256，4K = 81,024。
    // 0xAC 是 FPGA 发布的每帧字节数；这里不写死任何几何。
    let expect_tlp = if st.ddr_len != 0 && st.ddr_len % 128 == 0 {
        st.ddr_len / 128
    } else {
        0
    };
    println!(
        "推帧    : {}  0xC0 已推 {} 帧，最近一帧 {} 笔 MWr（0xAC={}B / 128B = {}），0xC4=0x{:08x}（busy={} last_ok={} 门铃已发={}）",
        if st.push_en != 0 { "已初始化（CTRL bit6=1）" } else { "**未初始化**（CTRL bit6=0）" },
        st.pushed,
        st.push_tlp_cnt,
        st.ddr_len,
        expect_tlp,
        st.push_status,
        bit(st.push_status, ANLOGIC_PUSH_STATUS_BUSY),
        bit(st.push_status, ANLOGIC_PUSH_STATUS_LAST_OK),
        bit(st.push_status, ANLOGIC_PUSH_STATUS_DB_SENT)
    );
    if st.pushed != 0 && expect_tlp != 0 && st.push_tlp_cnt != expect_tlp {
        eprintln!(
            "!! 最近一帧 {} 笔 MWr != 0xAC/128 = {}：检查 FPGA 的帧长与 128B MWr 约定",
            st.push_tlp_cnt, expect_tlp
        );
    }
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
    println!(
        "收帧缓冲: mailbox seq={} count={}（驱动按门铃补齐）",
        st.seq, st.count
    );
    if !quiet {
        println!(
            "CTRL    : 0x{:08x}（使能={} 触发={} 重触发电平={} 推帧取B路={} DDR搬运={} 推帧={} 推帧源={}）",
            st.ctrl,
            bit(st.ctrl, ANLOGIC_CTRL_ENABLE),
            bit(st.ctrl, 2),
            bit(st.ctrl, ANLOGIC_CTRL_MCLK_SEL),
            bit(st.ctrl, ANLOGIC_CTRL_CAM_B),
            bit(st.ctrl, ANLOGIC_CTRL_DDR_EN),
            bit(st.ctrl, ANLOGIC_CTRL_PUSH_EN),
            bit(st.ctrl, ANLOGIC_CTRL_PUSH_SRC)
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
    unsafe {
        sys::ioctl_ptr(
            fd,
            ANLOGIC_IOCPUSHCFG,
            &mut cfg as *mut IocPushCfg as *mut c_void,
        )
    }?;
    Ok(cfg)
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
        if cfg.applied == 0 {
            "无（只复核）"
        } else {
            "已写并回读复核通过"
        },
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
        "用法: {} [选项] [status|retrig|push|arm|ack|sync [start|abort|status]]\n\
         \n\
         查/改 PH1P100 的相机链路与推帧通路（除 arm/ack 写 CTRL 外只读；不 mmap BAR）。\n\
         \x20 status  查链路与推帧状态（默认）\n\
         \x20 retrig  手动重新触发一次相机初始化（链路掉了之后恢复用）\n\
         \x20 push    复核推帧目标/门铃地址；--dst/--db 必须与驱动分配的槽 0/门铃完全一致\n\
         \x20 arm     选 A/B 路并申请一个 one-shot 额度（-r B 选 B 路）\n\
         \x20 ack     释放当前 HELD（ACK 0xC0），并把 CTRL 收回 0x21\n\
         \x20 sync    硬件同步曝光：status（默认）/ start / abort（0x1E0..0x20C）\n\
         \n\
         选项:\n\
         \x20 -d <节点>     控制节点，默认 {}\n\
         \x20 -t <毫秒>     retrig 后最多等多久等锁定，默认 {}（0..{}，0 = 不等）\n\
         \x20 -r <A|B>      arm 选路，默认 A\n\
         \x20 --dst-lo <v>  推帧目标低 32 位（= 驱动槽 0 数据起点）\n\
         \x20 --dst-hi <v>  推帧目标地址高 32 位\n\
         \x20 --db-lo <v>   门铃低 32 位（= mailbox.push_dbell）\n\
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
        LongOpt {
            name: "dev",
            has_arg: true,
            val: b'd' as u32,
        },
        LongOpt {
            name: "timeout",
            has_arg: true,
            val: b't' as u32,
        },
        LongOpt {
            name: "dst-lo",
            has_arg: true,
            val: 2,
        },
        LongOpt {
            name: "dst-hi",
            has_arg: true,
            val: 3,
        },
        LongOpt {
            name: "db-lo",
            has_arg: true,
            val: 4,
        },
        LongOpt {
            name: "db-hi",
            has_arg: true,
            val: 5,
        },
        LongOpt {
            name: "route",
            has_arg: true,
            val: b'r' as u32,
        },
        LongOpt {
            name: "quiet",
            has_arg: false,
            val: b'q' as u32,
        },
        LongOpt {
            name: "help",
            has_arg: false,
            val: b'h' as u32,
        },
    ];

    let mut getopt = Getopt::new(&argv, "d:t:qhr:", &longopts);

    let mut dev_path = DEF_DEV.to_string();
    let mut wait_ms = DEF_WAIT_MS;
    let mut quiet = false;
    let mut dst_lo: Option<u32> = None;
    let mut dst_hi: Option<u32> = None;
    let mut db_lo: Option<u32> = None;
    let mut db_hi: Option<u32> = None;
    let mut route = "A".to_string();

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
                    _ => Err(format!(
                        "--timeout 需要 0..{} 的毫秒数（收到 '{}'）",
                        MAX_WAIT_MS, s
                    )),
                }
            }
            b'q' => {
                quiet = true;
                Ok(())
            }
            b'r' => {
                let s = arg.unwrap_or_default();
                if s.eq_ignore_ascii_case("A") || s.eq_ignore_ascii_case("B") {
                    route = s.to_uppercase();
                    Ok(())
                } else {
                    Err(format!("--route 只接受 A 或 B（收到 '{}'）", s))
                }
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
    let max_pos = if cmd == "sync" { 2 } else { 1 };
    if positional.len() > max_pos {
        eprintln!("!! 多余的参数：'{}'", positional[max_pos]);
        usage(&argv[0]);
        return EX_USAGE;
    }
    if !matches!(cmd, "status" | "retrig" | "push" | "arm" | "ack" | "sync") {
        eprintln!(
            "!! 不认识的动作 '{}'（只有 status / retrig / push / arm / ack / sync）",
            cmd
        );
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

    if cmd == "sync" {
        let action = positional.get(1).map(|s| s.as_str()).unwrap_or("status");
        let res = match action {
            "status" => sync_status(fd),
            "start" => sync_cmd(fd, true, wait_ms),
            "abort" => sync_cmd(fd, false, wait_ms),
            _ => {
                eprintln!("!! sync 只接受 status / start / abort（收到 '{}'）", action);
                usage(&argv[0]);
                return EX_USAGE;
            }
        };
        return match res {
            Ok(()) => EX_OK,
            Err(msg) => {
                eprintln!("!! {}", msg);
                EX_DEV
            }
        };
    }

    if cmd == "arm" {
        println!("=== ARM one-shot（{} 路）===", route);
        return match do_arm(fd, route == "B") {
            Ok(()) => EX_OK,
            Err(msg) => {
                eprintln!("!! {}", msg);
                EX_PUSH_ERR
            }
        };
    }

    if cmd == "ack" {
        println!("=== ACK / 释放 HELD ===");
        return match do_ack(fd) {
            Ok(()) => EX_OK,
            Err(msg) => {
                eprintln!("!! {}", msg);
                EX_PUSH_ERR
            }
        };
    }

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
            "  {} 路状态字 0x{:08x}，lane_valid=0x{:x}（四条全对齐 = 0x{:x}）、0x4C 帧计数 {}{}",
            if rt.ctrl & ANLOGIC_CTRL_CAM_B != 0 {
                "B"
            } else {
                "A"
            },
            rt.cam_stat,
            rt.lane_valid,
            ANLOGIC_CAM_LANE_ALL,
            rt.frame_cnt,
            if rt.frames_flowing != 0 {
                "（相对开始等时变过 ⇒ 帧已经在出）"
            } else {
                "（没动）"
            }
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
                /*
                 * 重触发返回时 lane 可能还在逐步对齐；但如果复核时仍没锁、
                 * 且 0x4C 帧计数相对 ioctl 返回时没有再增加，就说明这次
                 * 重触发没有把链路救回来，别用 0 骗脚本。
                 */
                if st.lane_valid != ANLOGIC_CAM_LANE_ALL && st.frame_cnt == rt.frame_cnt {
                    eprintln!("!! 重触发后链路仍未锁，且 0x4C 帧计数没有再增加");
                    eprintln!("   （传感器/接线/供电/参考时钟侧；可再跑 retrig 或 status 复核）");
                    return EX_NO_LOCK;
                }
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
                eprintln!("   （-EIO：写进去读回不一致 / 读到全 F；-EINVAL：flags 有未知位，或地址不是驱动分配的槽 0/门铃）");
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
