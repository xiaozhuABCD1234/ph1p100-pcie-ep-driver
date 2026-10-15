// SPDX-License-Identifier: GPL-2.0
/*!
 * pci_bar —— PH1P100 EP 控制节点最小用法示例（原 app/pci_bar.c 的 Rust 版）
 *
 * 通过 /dev/ANLOGIC-PCI0_control 的 ANLOGIC_IOCW / ANLOGIC_IOCR 对某个 BAR 的
 * 某个偏移做单 dw 读或写。
 *
 * 用法：
 *   pci_bar <bar_id 0..5> <offset>            # 读
 *   pci_bar <bar_id 0..5> <offset> <wdata>    # 写
 *
 * BAR1/BAR3/BAR5 没有各自的映射（BAR1 是 I/O BAR，BAR3/BAR5 是 64 位 BAR 的高半），
 * 驱动里对应 hw->bar[] 为 NULL，这里直接拒掉（规约 15）。
 *
 * 退出码沿用 C 版：用法/参数错是 `return -1`（进程退出码 255），open 失败是 1，其余 0。
 */
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;

use anlogic_pci_tools::ioctl::{IocBar, ANLOGIC_IOCR, ANLOGIC_IOCW};
use anlogic_pci_tools::parse::strtoul_base0_prefix;
use anlogic_pci_tools::sys;

const CTRL_DEV: &str = "/dev/ANLOGIC-PCI0_control";

/// 对应 C 版的 FATAL 宏：行号/文件名用 Rust 侧的 `line!()`/`file!()`。
macro_rules! fatal {
    ($e:expr) => {{
        eprintln!(
            "Error at line {}, file {} ({}) [{}]",
            line!(),
            file!(),
            $e.raw_os_error().unwrap_or(0),
            sys::errno_str(&$e)
        );
        std::process::exit(1);
    }};
}

// 与 C 版一样：返回值（ioctl 的结果）不使用。
fn reg_write(fd: i32, bar_id: u32, offset: u32, wdata: u32) {
    let mut bar = IocBar {
        bar_id: bar_id as u16,
        bar_type: 0,
        bar_offaddr: offset,
        wdata,
        rdata: 0,
    };
    // SAFETY: bar 就是 ANLOGIC_IOCW 对应的结构体
    let _ = unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCW, &mut bar as *mut IocBar as *mut std::ffi::c_void) };
}

fn reg_read(fd: i32, bar_id: u32, offset: u32) -> u32 {
    let mut bar = IocBar {
        bar_id: bar_id as u16,
        bar_type: 0,
        bar_offaddr: offset,
        wdata: 0,
        rdata: 0,
    };
    // SAFETY: bar 就是 ANLOGIC_IOCR 对应的结构体
    let _ = unsafe { sys::ioctl_ptr(fd, ANLOGIC_IOCR, &mut bar as *mut IocBar as *mut std::ffi::c_void) };
    bar.rdata
}

fn main() {
    let argv: Vec<String> = std::env::args_os()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();

    if argv.len() < 3 || argv.len() > 4 {
        println!("Usage: {} <bar_id 0..5> <offset> [wdata]", argv[0]);
        anlogic_pci_tools::exit_with(-1);
    }

    // C 版是 `uint32_t bar_id = strtoul(argv[1], 0, 0);`：不检查 end，高位截断
    let bar_id = strtoul_base0_prefix(&argv[1]).0 as u32;
    if bar_id >= 6 {
        println!("Invalid bar ID, must be 0..5.");
        anlogic_pci_tools::exit_with(-1);
    }

    // BAR1/BAR3/BAR5 没有独立映射，驱动会在 ioctl 里返回 -EINVAL，这里先挡掉
    if bar_id == 1 || bar_id == 3 || bar_id == 5 {
        println!("BAR{} is not independently mapped (no dedicated resource).", bar_id);
        anlogic_pci_tools::exit_with(-1);
    }

    let offset = strtoul_base0_prefix(&argv[2]).0 as u32;
    if offset % 4 != 0 {
        println!("Offset must be 4byte aligned.");
        anlogic_pci_tools::exit_with(-1);
    }

    let mut wrdata = 0u32;
    let mode;
    if argv.len() == 4 {
        wrdata = strtoul_base0_prefix(&argv[3]).0 as u32;
        println!("Pci bar{}: address: 0x{:x} write data: 0x{:x}.", bar_id, offset, wrdata);
        mode = 1;
    } else {
        mode = 2;
        println!("Pci bar{}: address: 0x{:x} read.", bar_id, offset);
    }

    // open(CTRL_DEV, O_RDWR | O_SYNC)
    let file = match std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(sys::O_SYNC)
        .open(CTRL_DEV)
    {
        Ok(f) => f,
        Err(e) => fatal!(e),
    };
    println!("character device {} opened.", CTRL_DEV);

    if mode == 1 {
        reg_write(file.as_raw_fd(), bar_id, offset, wrdata);
        println!("Write complete.");
    } else {
        let reg_value = reg_read(file.as_raw_fd(), bar_id, offset);
        println!("Read data: 0x{:x} complete.", reg_value);
    }

    // file 离开作用域即 close(fd)
    anlogic_pci_tools::exit_with(0);
}
