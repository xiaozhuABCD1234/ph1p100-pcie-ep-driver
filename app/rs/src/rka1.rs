// SPDX-License-Identifier: GPL-2.0
//! T07：RKA1 安全扩展的用户态配套（与 `include/anlogic_pci_ioctl.h` 里
//! `struct anlogic_ioc_rka1` / `ANLOGIC_IOCRKA1` 一一对应）。
//!
//! 两块内容：
//!   1) [`Rka1`]：扩展探测 / 授权配置 / one-shot ARM / ACK_RELEASE / CANCEL 的薄封装。
//!      所有操作都 fail closed：EXT_MAGIC != "RKA1" 时直接报错，**不退回旧行为**。
//!   2) [`r10f`]：R10F 非压缩帧的严格解析（magic/version/format/codec/几何/三层长度/
//!      三段 CRC/尾零/flags），任何一项不符就拒绝，绝不「夹紧后当成功」。
//!
//! 说明：本模块不改任何既有结构体布局与 ioctl 编号；新结构体只走新编号 15。
//!
//! **T11 增量（本文件相对 T07 版本）**：`r10f` 模块的字段布局此前是按早期草案写的
//! （magic "R0FR"、偏移/宽度不匹配正式帧），会拒绝 FPGA 实际输出的每一帧。本增量
//! 把它改为与 `contract/acceptance-vectors.json` +
//! `references/local-inputs/parallel/frame/frame_contract.py` + T11 写者 RTL
//! (`fpga/csi2/pcie_integration/rtl/ep_side/ddr/cam_r10f_writer.v`) 逐字段一致的正式
//! 布局（magic "R10F"、version 1.0、header_bytes 128、codec_id=0 …），并补齐
//! reserved/flags/几何自洽检查；离线对照实现见 `tools/pcie_integration/r10f_strict.py`。

use crate::ioctl::*;
use crate::sys;

/// `struct anlogic_ioc_rka1`（与内核头文件逐字段对应）
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct IocRka1 {
    pub op: u32,
    pub cap_bytes: u32,
    pub session: u32,
    pub ack_seq: u32,
    pub ack_session: u32,
    pub ext_magic: u32,
    pub ext_version: u32,
    pub tx_status: u32,
    pub push_err: u32,
    pub push_status: u32,
    pub push_done: u32,
    pub armed: u32,
    pub held: u32,
    pub busy: u32,
    pub rejected: u32,
}

pub const RKA1_OP_PROBE: u32 = 0;
pub const RKA1_OP_CONFIGURE: u32 = 1;
pub const RKA1_OP_ARM: u32 = 2;
pub const RKA1_OP_ACK: u32 = 3;
pub const RKA1_OP_CANCEL: u32 = 4;

pub const RKA1_EXT_MAGIC: u32 = 0x3141_4B52; // "RKA1"（小端）
pub const RKA1_EXT_VERSION_MAJOR: u32 = 1;

pub const RKA1_ST_ARMED: u32 = 0x01;
pub const RKA1_ST_BUSY: u32 = 0x02;
pub const RKA1_ST_HELD: u32 = 0x04;
pub const RKA1_ST_REJECT: u32 = 0x08;
pub const RKA1_ST_DRAINING: u32 = 0x10;

/// `_IOWR('x', 15, struct anlogic_ioc_rka1)`（与头文件一致；Rust 侧按 ABI 重算）
pub const ANLOGIC_IOCRKA1: u64 = iowr(15, core::mem::size_of::<IocRka1>() as u32);

const fn iowr(nr: u32, size: u32) -> u64 {
    ((3u64) << 30) | ((size as u64) << 16) | ((ANLOGIC_IOC_MAGIC as u64) << 8) | (nr as u64)
}

pub struct Rka1 {
    fd: i32,
}

impl Rka1 {
    pub fn new(fd: i32) -> Self {
        Rka1 { fd }
    }

    fn call(&self, rq: &mut IocRka1) -> Result<(), (i32, String)> {
        let rv = unsafe { sys::ioctl(self.fd, ANLOGIC_IOCRKA1, rq as *mut IocRka1) };
        if rv < 0 {
            let err = sys::errno();
            return Err((err, describe(rq, err)));
        }
        Ok(())
    }

    /// 探测：EXT_MAGIC/EXT_VERSION 必须对。失败 = 该 FPGA 映像没有安全扩展。
    pub fn probe(&self) -> Result<IocRka1, (i32, String)> {
        let mut rq = IocRka1 { op: RKA1_OP_PROBE, ..Default::default() };
        self.call(&mut rq)?;
        if rq.ext_magic != RKA1_EXT_MAGIC {
            return Err((
                libc_einval(),
                format!(
                    "EXT_MAGIC=0x{:08x} != 0x{:08x}（旧映像）：拒绝进入安全模式",
                    rq.ext_magic, RKA1_EXT_MAGIC
                ),
            ));
        }
        if (rq.ext_version >> 16) != RKA1_EXT_VERSION_MAJOR {
            return Err((
                libc_einval(),
                format!("EXT_VERSION=0x{:08x} 主版本不符", rq.ext_version),
            ));
        }
        Ok(rq)
    }

    pub fn configure(&self, cap_bytes: u32, session: u32) -> Result<IocRka1, (i32, String)> {
        let mut rq = IocRka1 {
            op: RKA1_OP_CONFIGURE,
            cap_bytes,
            session,
            ..Default::default()
        };
        self.call(&mut rq)?;
        Ok(rq)
    }

    pub fn arm(&self) -> Result<IocRka1, (i32, String)> {
        let mut rq = IocRka1 { op: RKA1_OP_ARM, ..Default::default() };
        self.call(&mut rq)?;
        if rq.armed == 0 {
            return Err((libc_einval(), format!("ARM 后 TX_STATUS=0x{:08x} 未置 armed", rq.tx_status)));
        }
        Ok(rq)
    }

    pub fn ack(&self, seq: u32, session: u32) -> Result<IocRka1, (i32, String)> {
        let mut rq = IocRka1 {
            op: RKA1_OP_ACK,
            ack_seq: seq,
            ack_session: session,
            ..Default::default()
        };
        self.call(&mut rq)?;
        if rq.held != 0 {
            return Err((libc_einval(), format!("ACK 后仍 held（TX_STATUS=0x{:08x}）", rq.tx_status)));
        }
        Ok(rq)
    }

    pub fn cancel(&self) -> Result<IocRka1, (i32, String)> {
        let mut rq = IocRka1 { op: RKA1_OP_CANCEL, ..Default::default() };
        self.call(&mut rq)?;
        Ok(rq)
    }
}

fn libc_einval() -> i32 {
    22
}

fn describe(rq: &IocRka1, err: i32) -> String {
    format!(
        "RKA1 ioctl(op={}) 失败 errno={}（TX_STATUS=0x{:08x}，0xC8={}）",
        rq.op, err, rq.tx_status, rq.push_err
    )
}

/// R10F 非压缩帧的严格解析（T07 引入；**T11 修正为正式布局**）。
///
/// 判据（任一项不符即 `Err`，不做任何"夹紧/容错"）：
///   · magic == "R10F"（小端读 0x46303152）、version == 1.0（0x04 主 / 0x06 次）、
///     header_bytes == 128；
///   · format_id == 0（非压缩，0x0B）、codec_id == 0（0x0C；BPR1 是别的 codec，
///     不在这里）、valid_bits == 10（0x2A）、packing == 0（0x2B）；
///   · 几何自洽：width（0x1C）>0 且 %4==0、stride（0x20）== width*5/4、
///     raw_bytes（0x2C）== payload_bytes（0x30）== stride*height；
///   · flags（0x4C）不得含 BAD(1)/PARTIAL(4)/OVERSIZE(8)；CROPPED(2) 允许；
///   · reserved 0x5C..0x7F 必须全 0；
///   · payload 长度精确等于 payload_bytes、transport_tail 必须全 0；
///   · raw_crc32 == payload_crc32 == CRC32(packed 载荷)（IEEE 反射 poly 0xEDB88320，
///     init/final 全 1）；
///   · header_crc32（0x58）== CRC32(头 128B，其中 0x58..0x5B 先清 0)；
///   · expect_session 给出时 session（0x10）必须相等。
pub mod r10f {
    pub const MAGIC: u32 = 0x4630_3152; // 'R','1','0','F'（小端读）
    pub const HDR_BYTES: usize = 128;
    pub const VERSION_MAJOR: u16 = 1;
    pub const VERSION_MINOR: u16 = 0;
    pub const FORMAT_RAW10_PACKED: u16 = 0;
    pub const CODEC_NONE: u16 = 0;
    pub const VALID_BITS: u8 = 10;

    pub const FLAG_BAD: u32 = 1 << 0;
    pub const FLAG_CROPPED: u32 = 1 << 1;
    pub const FLAG_PARTIAL: u32 = 1 << 2;
    pub const FLAG_OVERSIZE: u32 = 1 << 3;

    #[derive(Debug, Clone, Copy)]
    pub struct R10fHeader {
        pub version_major: u16,
        pub version_minor: u16,
        pub header_bytes: u16,
        pub camera_id: u8,
        pub format_id: u8,
        pub codec_id: u32,
        pub session_id: u32,
        pub capture_seq: u32,
        pub mode_id: u32,
        pub width: u16,
        pub height: u16,
        pub stride: u32,
        pub roi_x: u16,
        pub roi_y: u16,
        pub bayer_phase: u8,
        pub bayer_dir: u8,
        pub valid_bits: u8,
        pub packing: u8,
        pub raw_bytes: u32,
        pub payload_bytes: u32,
        pub block_count: u32,
        pub flags: u32,
        pub raw_crc32: u32,
        pub payload_crc32: u32,
        pub header_crc32: u32,
    }

    pub fn le16(b: &[u8], o: usize) -> u16 {
        u16::from_le_bytes([b[o], b[o + 1]])
    }
    pub fn le32(b: &[u8], o: usize) -> u32 {
        u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
    }

    pub fn crc32_ieee(data: &[u8]) -> u32 {
        let mut crc: u32 = 0xffff_ffff;
        for &byte in data {
            crc ^= byte as u32;
            for _ in 0..8 {
                let lsb = crc & 1;
                crc >>= 1;
                if lsb != 0 {
                    crc ^= 0xEDB8_8320;
                }
            }
        }
        !crc
    }

    /// 严格解析。`hdr` 至少 128B（只取前 128B）；`payload` 为 packed 有效载荷；
    /// `transport_tail` 为 4KiB 对齐尾（content_bytes 之后到 dma_bytes）。
    pub fn parse(hdr: &[u8], payload: &[u8], transport_tail: &[u8],
                 expect_w: u16, expect_h: u16, expect_session: Option<u32>)
        -> Result<R10fHeader, String>
    {
        if hdr.len() < HDR_BYTES {
            return Err(format!("header too short: {} < {}", hdr.len(), HDR_BYTES));
        }
        if le32(hdr, 0) != MAGIC {
            return Err(format!("bad R10F magic 0x{:08x} != 0x{:08x}", le32(hdr, 0), MAGIC));
        }
        let h = R10fHeader {
            version_major: le16(hdr, 0x04),
            version_minor: le16(hdr, 0x06),
            header_bytes: le16(hdr, 0x08),
            camera_id: hdr[0x0A],
            format_id: hdr[0x0B],
            codec_id: le32(hdr, 0x0C),
            session_id: le32(hdr, 0x10),
            capture_seq: le32(hdr, 0x14),
            mode_id: le32(hdr, 0x18),
            width: le16(hdr, 0x1C),
            height: le16(hdr, 0x1E),
            stride: le32(hdr, 0x20),
            roi_x: le16(hdr, 0x24),
            roi_y: le16(hdr, 0x26),
            bayer_phase: hdr[0x28],
            bayer_dir: hdr[0x29],
            valid_bits: hdr[0x2A],
            packing: hdr[0x2B],
            raw_bytes: le32(hdr, 0x2C),
            payload_bytes: le32(hdr, 0x30),
            block_count: le32(hdr, 0x34),
            flags: le32(hdr, 0x4C),
            raw_crc32: le32(hdr, 0x50),
            payload_crc32: le32(hdr, 0x54),
            header_crc32: le32(hdr, 0x58),
        };
        if (h.version_major, h.version_minor) != (VERSION_MAJOR, VERSION_MINOR) {
            return Err(format!("R10F version {}.{} != 1.0", h.version_major, h.version_minor));
        }
        if h.header_bytes != HDR_BYTES as u16 {
            return Err(format!("R10F header_bytes {} != 128", h.header_bytes));
        }
        if h.format_id != FORMAT_RAW10_PACKED as u8 {
            return Err(format!("R10F format_id {} != 0（非压缩）", h.format_id));
        }
        // codec_id 是 u32 字段（0x0C）：正式非压缩帧必须为 0
        if h.codec_id != CODEC_NONE as u32 {
            return Err(format!("R10F codec_id {} 未知（BPR1 走别的 codec）", h.codec_id));
        }
        if h.valid_bits != VALID_BITS || h.packing != 0 {
            return Err(format!(
                "R10F valid_bits={} / packing={}（本框架只支持 10bit packed）",
                h.valid_bits, h.packing
            ));
        }
        if h.width == 0 || h.height == 0 || (h.width & 3) != 0 {
            return Err(format!("R10F geometry {}x{} 非法（width 必须为 4 的倍数）", h.width, h.height));
        }
        let expect_stride = (h.width as u32) / 4 * 5;
        if h.stride != expect_stride {
            return Err(format!("R10F stride {} != width*5/4 {}", h.stride, expect_stride));
        }
        if h.raw_bytes != h.payload_bytes {
            return Err(format!("R10F raw_bytes {} != payload_bytes {}", h.raw_bytes, h.payload_bytes));
        }
        if h.payload_bytes != h.stride * h.height as u32 {
            return Err(format!(
                "R10F payload {} != stride {} * height {}",
                h.payload_bytes, h.stride, h.height
            ));
        }
        if h.flags & (FLAG_BAD | FLAG_PARTIAL | FLAG_OVERSIZE) != 0 || h.flags & !FLAG_CROPPED != 0 {
            return Err(format!("R10F flags 0x{:x} 不可发布（只允许 0/2=CROPPED）", h.flags));
        }
        if h.block_count < 1 {
            return Err(format!("R10F block_count {} < 1", h.block_count));
        }
        if hdr[0x5C..HDR_BYTES].iter().any(|b| *b != 0) {
            return Err("R10F reserved 0x5C..0x7F 非 0".to_string());
        }
        if h.width != expect_w || h.height != expect_h {
            return Err(format!(
                "R10F geometry {}x{} != 已授权的 {}x{}",
                h.width, h.height, expect_w, expect_h
            ));
        }
        if payload.len() != h.payload_bytes as usize {
            return Err(format!(
                "payload len {} != payload_bytes {}",
                payload.len(), h.payload_bytes
            ));
        }
        if !transport_tail.iter().all(|b| *b == 0) {
            return Err("transport tail is not all zero".to_string());
        }
        if let Some(sess) = expect_session {
            if h.session_id != sess {
                return Err(format!("R10F session 0x{:08x} != 授权 session 0x{:08x}", h.session_id, sess));
            }
        }
        // header CRC：把自身 crc 字段清 0 再算
        let mut hdr_copy = [0u8; HDR_BYTES];
        hdr_copy.copy_from_slice(&hdr[..HDR_BYTES]);
        hdr_copy[0x58] = 0;
        hdr_copy[0x59] = 0;
        hdr_copy[0x5a] = 0;
        hdr_copy[0x5b] = 0;
        let hc = crc32_ieee(&hdr_copy);
        if hc != h.header_crc32 {
            return Err(format!("header_crc32 0x{:08x} != 计算值 0x{:08x}", h.header_crc32, hc));
        }
        let pc = crc32_ieee(payload);
        if pc != h.payload_crc32 {
            return Err(format!("payload_crc32 0x{:08x} != 计算值 0x{:08x}", h.payload_crc32, pc));
        }
        if pc != h.raw_crc32 {
            return Err(format!(
                "raw_crc32 0x{:08x} != payload_crc32 0x{:08x}（非压缩必须相等）",
                h.raw_crc32, h.payload_crc32
            ));
        }
        Ok(h)
    }
}
