// SPDX-License-Identifier: GPL-2.0
//! PH1P100 PCIe EP（RK3588 主机侧）用户态工具的共享层。
//!
//! 三个（加链路自检共四个）工具都复用了这里的三块东西：
//!   * [`ioctl`]：ioctl 协议常量与结构体，逐项对照 `include/anlogic_pci_ioctl.h`
//!     （该头文件是内核与用户态共用的唯一一份，本文件只是把它在 Rust 侧显式声明一遍，
//!     不改动它的任何语义；没有用 bindgen 之类需要额外工具链的做法）；
//!   * [`sys`]：少数几个必须自己声明的 libc 入口（ioctl/mmap/flock/strerror），
//!     文件与目录读写一律用 std；
//!   * [`getopt`]：原 C 版用的 getopt_long(3) 的等价实现；
//!   * [`parse`]：原 C 版用的 strtoul(3) / fscanf("%x") 的等价实现。

pub mod getopt;
pub mod ioctl;
pub mod parse;
pub mod rka1;
pub mod sys;

/// 与 C 版一样按退出码结束进程；退出前把 stdout 冲干净（process::exit 不跑析构）。
pub fn exit_with(code: i32) -> ! {
    use std::io::Write;
    let _ = std::io::stdout().flush();
    std::process::exit(code);
}
