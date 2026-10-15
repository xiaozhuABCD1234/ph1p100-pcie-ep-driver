// SPDX-License-Identifier: GPL-2.0
/*!
 * pcie_check —— PH1P100 EP（RK3588 主机侧）链路与落位自检
 * （原 app/pcie_check.c 的 Rust 版，功能不变）
 *
 * 只读 sysfs，不需要 root、不需要 lspci、不碰设备寄存器，也不要求驱动已加载：
 *   1. EP 有没有被枚举出来（按 vendor:device = 1edb:abcd 认）
 *   2. 当前/最大链路速率与宽度
 *   3. BAR 资源表（对照期望：BAR0/2/4 = 1 MB mem、BAR1 = 256 B I/O）
 *   4. 驱动是否加载、/dev 节点是否在
 *
 * 用法：
 *   pcie_check [bdf]      bdf 形如 0000:01:00.0，缺省则扫描全部 PCI 设备
 * 退出码：0 找到 EP；1 没找到。
 *
 * 想看内核枚举/AER 日志请另行执行：
 *   dmesg | grep -iE '1edb|pcie.*link|aer'
 */
use std::path::{Path, PathBuf};

use anlogic_pci_tools::parse::hex_prefix_u64;

const EP_VENDOR_ID: u32 = 0x1edb;
const EP_DEVICE_ID: u32 = 0xabcd;

const DEVPATH_MAX: usize = 256;
const DEVICES_DIR: &str = "/sys/bus/pci/devices";

/// 对应 C 版的 read_hex_file()：读 /sys/bus/pci/devices/<dir>/<name>，按 %x 解析。
fn read_hex_file(dir: &str, name: &str) -> Option<u32> {
    let path = format!("{}/{}/{}", DEVICES_DIR, dir, name);
    if path.len() >= DEVPATH_MAX {
        return None; // C 里 snprintf 溢出即失败
    }
    let s = std::fs::read_to_string(path).ok()?;
    hex_prefix_u64(&s).map(|v| v as u32) // C 里赋给 unsigned（截断）
}

/// 对应 C 版的 read_str_file()：读一行，去掉结尾的换行/空格。
fn read_str_file(dir: &str, name: &str) -> Option<String> {
    let path = format!("{}/{}/{}", DEVICES_DIR, dir, name);
    if path.len() >= DEVPATH_MAX {
        return None;
    }
    let s = std::fs::read_to_string(path).ok()?;
    let line = s.lines().next()?;
    Some(line.trim_end_matches(['\n', ' ']).to_string())
}

/// 对应 C 版的 show_bars()：把 resource 文件里前 6 行（start end flags）打出来。
fn show_bars(bdf: &str) {
    let path = format!("{}/{}/resource", DEVICES_DIR, bdf);
    if path.len() >= DEVPATH_MAX {
        return;
    }
    let content = match std::fs::read_to_string(&path) {
        Ok(c) => c,
        Err(_) => {
            println!("    (读不到 resource)");
            return;
        }
    };

    let mut i = 0usize;
    for line in content.lines() {
        if i >= 6 {
            break;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 3 {
            break; // fscanf 没凑够 3 个字段就停了
        }
        let (start, end, flags) = match (
            hex_prefix_u64(f[0]),
            hex_prefix_u64(f[1]),
            hex_prefix_u64(f[2]),
        ) {
            (Some(a), Some(b), Some(c)) => (a, b, c),
            _ => break,
        };

        if start == 0 && end == 0 && flags == 0 {
            println!("    BAR{}  (未使用)", i);
        } else {
            let kind = if flags & 0x100 != 0 { "I/O" } else { "mem" };
            println!(
                "    BAR{}  {:<3} 0x{:010x}-0x{:010x}  {} bytes",
                i,
                kind,
                start,
                end,
                end - start + 1
            );
        }
        i += 1;
    }
}

/// 对应 C 版的 check_one()：返回 Some(()) 表示这个 BDF 就是 PH1P100 EP。
fn check_one(bdf: &str) -> bool {
    println!("\n  BDF {}", bdf);

    let (vid, did) = match (read_hex_file(bdf, "vendor"), read_hex_file(bdf, "device")) {
        (Some(v), Some(d)) => (v, d),
        _ => {
            println!("    (读不到 vendor/device)");
            return false;
        }
    };
    let is_ep = vid == EP_VENDOR_ID && did == EP_DEVICE_ID;
    println!(
        "    PCI ID: {:04x}:{:04x}  {}",
        vid,
        did,
        if is_ep { "(PH1P100 EP ✓)" } else { "(不是 PH1P100 EP)" }
    );

    let link = format!("{}/{}/driver", DEVICES_DIR, bdf);
    if link.len() < DEVPATH_MAX {
        let driver = std::fs::read_link(PathBuf::from(&link));
        match driver {
            Ok(target) => {
                let name = target
                    .file_name()
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_else(|| target.to_string_lossy().into_owned());
                println!("    驱动: {}", name);
            }
            Err(_) => println!("    驱动: （未绑定）"),
        }
    }

    let (speed, max_width) = (read_str_file(bdf, "max_link_speed"), read_hex_file(bdf, "max_link_width"));
    if let (Some(s), Some(w)) = (speed, max_width) {
        println!("    LnkCap: Speed {}, Width x{}", s.trim_end(), w);
    }
    let (speed, width) = (read_str_file(bdf, "current_link_speed"), read_hex_file(bdf, "current_link_width"));
    if let (Some(s), Some(w)) = (speed, width) {
        println!("    LnkSta: Speed {}, Width x{}   (期望 5.0 GT/s PCIe, x2)", s.trim_end(), w);
    }

    println!("    BAR 资源:");
    show_bars(bdf);

    if let Some(b) = read_str_file(bdf, "modalias") {
        println!("    modalias: {}", b);
    }

    is_ep
}

fn main() {
    let argv: Vec<String> = std::env::args_os()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();

    let mut found = false;
    let mut want: Option<String> = None;

    if argv.len() > 1 {
        if argv[1] == "-h" || argv[1] == "--help" {
            println!("Usage: {} [bdf]", argv[0]);
            anlogic_pci_tools::exit_with(0);
        }
        want = Some(argv[1].clone());
    }

    println!("== PH1P100 EP 链路自检（只读 sysfs，不需要 root）==");

    match want {
        Some(bdf) => found = check_one(&bdf),
        None => {
            let entries = match std::fs::read_dir(DEVICES_DIR) {
                Ok(e) => e,
                Err(e) => {
                    eprintln!("opendir {}: {}", DEVICES_DIR, anlogic_pci_tools::sys::errno_str(&e));
                    anlogic_pci_tools::exit_with(1);
                }
            };
            for de in entries.flatten() {
                let name = de.file_name().to_string_lossy().into_owned();
                if name.starts_with('.') {
                    continue;
                }
                let (vid, did) = match (read_hex_file(&name, "vendor"), read_hex_file(&name, "device")) {
                    (Some(v), Some(d)) => (v, d),
                    _ => continue,
                };
                if vid == EP_VENDOR_ID && did == EP_DEVICE_ID {
                    check_one(&name);
                    found = true;
                }
            }
        }
    }

    println!("\n== 驱动与设备节点 ==");
    println!(
        "    /sys/module/anlogic_pci      : {}",
        if Path::new("/sys/module/anlogic_pci").exists() { "已加载" } else { "未加载" }
    );
    println!(
        "    /dev/ANLOGIC-PCI0_control    : {}",
        if Path::new("/dev/ANLOGIC-PCI0_control").exists() { "存在" } else { "不存在" }
    );
    println!(
        "    /dev/ANLOGIC-PCI0_user       : {}",
        if Path::new("/dev/ANLOGIC-PCI0_user").exists() { "存在" } else { "不存在" }
    );

    if !found {
        println!("\n!! 没找到 {:04x}:* 设备 —— EP 未枚举", EP_VENDOR_ID);
        println!("   排查：FPGA 是否已配置/固化？12V 供电？转接卡插紧？");
        println!("   重扫：echo 1 > /sys/bus/pci/rescan");
        println!("   日志：dmesg | grep -iE 'pcie.*(link|fail|error)'");
        anlogic_pci_tools::exit_with(1);
    }

    println!("\n链路与 BAR 若符合期望 = 硬件/建链正常；/dev 节点需要先 insmod anlogic_pci.ko。");
    anlogic_pci_tools::exit_with(0);
}
