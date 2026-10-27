// SPDX-License-Identifier: GPL-2.0
/*!
 * r10f_check —— T11.rk-align-raw：R10F v1 非压缩帧的**离线严格校验**（RK 侧）
 *
 * 输入：一份「R10F 传输内容」文件（= 主机槽内 `hdr_bytes` 偏移起、长度 `dma_bytes`
 * 的那段字节：128B 头 + packed 有效载荷 + 4KiB 对齐全 0 尾）。
 *   来源可以是：
 *     · FPGA 实收帧的槽内容 dump（`dd if=/dev/... skip=<槽偏移 + hdr_bytes> count=<dma_bytes>`）；
 *     · PC 侧生成/发布的样例（tools/pcie_integration/r10f_strict.py gen 的输出）。
 *
 * 判据与 `app/rs/src/rka1.rs` 的 `r10f::parse`、Python 对照工具、合同验收向量
 * (`contract/acceptance-vectors.json`) 逐字段一致；任何一项不符 → 退出码 5、不打印 PASS。
 *
 * 用法：
 *   r10f_check <文件> [--session 0x20261017] [--width 1920 --height 1080]
 * 退出码：0 校验通过；1 用法错；2 文件读不了；5 严格校验不过。
 */
use std::io::Read;

use anlogic_pci_tools::rka1::r10f;

const EX_OK: i32 = 0;
const EX_USAGE: i32 = 1;
const EX_IO: i32 = 2;
const EX_INCONSISTENT: i32 = 5;

fn usage(prog: &str) {
    eprintln!(
        "用法：{} <R10F内容文件> [--session 0xNNNNNNNN] [--width W --height H]\n\
         \x20 输入 = 槽内 hdr_bytes 偏移起、长度 dma_bytes 的一段（128B 头+载荷+尾零）\n\
         \x20 退出码：0 通过，1 用法，2 读文件失败，5 严格校验不过",
        prog
    );
}

fn parse_u32(s: &str) -> Option<u32> {
    let t = s.trim();
    if let Some(h) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u32::from_str_radix(h, 16).ok()
    } else {
        t.parse::<u32>().ok()
    }
}

fn run() -> i32 {
    let argv: Vec<String> = std::env::args().collect();
    if argv.len() < 2 {
        usage(&argv[0]);
        return EX_USAGE;
    }
    let path = argv[1].clone();
    let mut expect_session: Option<u32> = None;
    let mut expect_w: Option<u16> = None;
    let mut expect_h: Option<u16> = None;

    let mut i = 2;
    while i < argv.len() {
        match argv[i].as_str() {
            "--session" if i + 1 < argv.len() => {
                match parse_u32(&argv[i + 1]) {
                    Some(v) => expect_session = Some(v),
                    None => {
                        eprintln!("!! --session 需要 32bit 数（可 0x 前缀）");
                        return EX_USAGE;
                    }
                }
                i += 2;
            }
            "--width" if i + 1 < argv.len() => {
                match parse_u32(&argv[i + 1]).and_then(|v| u16::try_from(v).ok()) {
                    Some(v) => expect_w = Some(v),
                    None => {
                        eprintln!("!! --width 需要 16bit 数");
                        return EX_USAGE;
                    }
                }
                i += 2;
            }
            "--height" if i + 1 < argv.len() => {
                match parse_u32(&argv[i + 1]).and_then(|v| u16::try_from(v).ok()) {
                    Some(v) => expect_h = Some(v),
                    None => {
                        eprintln!("!! --height 需要 16bit 数");
                        return EX_USAGE;
                    }
                }
                i += 2;
            }
            "-h" | "--help" => {
                usage(&argv[0]);
                return EX_OK;
            }
            other => {
                eprintln!("!! 未知参数 '{}'", other);
                usage(&argv[0]);
                return EX_USAGE;
            }
        }
    }

    let mut buf = Vec::new();
    match std::fs::File::open(&path).and_then(|mut f| f.read_to_end(&mut buf)) {
        Ok(_) => {}
        Err(e) => {
            eprintln!("!! 打不开/读不了 {}：{}", path, e);
            return EX_IO;
        }
    }
    if buf.len() < r10f::HDR_BYTES {
        eprintln!("!! 文件只有 {} 字节（R10F 头就要 128）", buf.len());
        return EX_INCONSISTENT;
    }
    if &buf[0..4] != b"R10F" {
        eprintln!("!! 不是 R10F 内容（magic = {:02x?}）", &buf[0..4]);
        return EX_INCONSISTENT;
    }

    let le16 = |o: usize| u16::from_le_bytes([buf[o], buf[o + 1]]);
    let le32 = |o: usize| u32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
    let width = le16(0x1C);
    let height = le16(0x1E);
    let payload_bytes = le32(0x30) as usize;
    let want = r10f::HDR_BYTES + payload_bytes;
    if buf.len() < want {
        eprintln!(
            "!! 文件长度 {} 小于 128+payload_bytes {}：截断",
            buf.len(),
            want
        );
        return EX_INCONSISTENT;
    }
    let (hdr, rest) = buf.split_at(r10f::HDR_BYTES);
    let (payload, tail) = rest.split_at(payload_bytes);

    let ew = expect_w.unwrap_or(width);
    let eh = expect_h.unwrap_or(height);
    match r10f::parse(hdr, payload, tail, ew, eh, expect_session) {
        Ok(h) => {
            println!(
                "R10F OK: {}x{} stride={} session=0x{:08x} seq={} mode={} camera={} flags=0x{:x} roi=({},{})",
                h.width, h.height, h.stride, h.session_id, h.capture_seq, h.mode_id,
                h.camera_id, h.flags, h.roi_x, h.roi_y
            );
            println!(
                "         raw_bytes={} payload_bytes={} crc(payload)=0x{:08x} crc(header)=0x{:08x} tail={}B 全零",
                h.raw_bytes, h.payload_bytes, h.payload_crc32, h.header_crc32,
                tail.len()
            );
            EX_OK
        }
        Err(e) => {
            eprintln!("R10F FAIL: {}（文件 {}，{} 字节）", e, path, buf.len());
            EX_INCONSISTENT
        }
    }
}

fn main() {
    anlogic_pci_tools::exit_with(run());
}
