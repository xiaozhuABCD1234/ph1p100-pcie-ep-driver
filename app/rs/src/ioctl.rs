// SPDX-License-Identifier: GPL-2.0
//! ioctl 协议（Rust 侧声明）—— 逐项对照 `include/anlogic_pci_ioctl.h`。
//!
//! 该头文件是内核与用户态共用的 ABI，**不在这里改动它的任何语义**：本文件只是把
//! 三个工具用到的那几条常量/结构体按同样的布局、同样的编号声明一遍，每一条都在
//! 注释里标出它对应头文件的哪一项。结构体要 `#[repr(C)]`，字段顺序/类型与头文件
//! 一字不差；末尾用编译期断言把 sizeof 与字段偏移钉死（等价于 C 版的 _Static_assert）。

#![allow(dead_code)]

// ---------------------------------------------------------------- ioctl 编码
//
// 对照头文件里的 #include <linux/ioctl.h>：_IOC(dir,type,nr,size) 的位域，
// arm64/x86_64 都是 asm-generic 这一套（dir 2 位 / size 14 位 / type 8 位 / nr 8 位）。
const IOC_NRBITS: u32 = 8;
const IOC_TYPEBITS: u32 = 8;
const IOC_SIZEBITS: u32 = 14;
const IOC_NRSHIFT: u32 = 0;
const IOC_TYPESHIFT: u32 = IOC_NRSHIFT + IOC_NRBITS;
const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + IOC_TYPEBITS;
const IOC_DIRSHIFT: u32 = IOC_SIZESHIFT + IOC_SIZEBITS;

const IOC_NONE: u32 = 0; // _IO
const IOC_WRITE: u32 = 1; // _IOW
const IOC_READ: u32 = 2; // _IOR

/// `_IOC(dir,type,nr,size)`；编号一律按 32 位无符号算，再放进 ioctl 的请求字。
const fn ioc(dir: u32, ty: u32, nr: u32, size: u32) -> u64 {
    ((dir << IOC_DIRSHIFT) | (ty << IOC_TYPESHIFT) | (nr << IOC_NRSHIFT) | (size << IOC_SIZESHIFT))
        as u64
}

/// `ANLOGIC_IOC_MAGIC = 'x'`
pub const ANLOGIC_IOC_MAGIC: u32 = b'x' as u32;
/// `ANLOGIC_XCL_MAGIC = 0X586C0C6C`（ANLOGIC_IOCINFO 入参里的魔数）
pub const ANLOGIC_XCL_MAGIC: u32 = 0x586C_0C6C;
/// `MAX_DATA_SIZE`（BULK 单次 ioctl 可带的上限 dw 数；驱动再把整段按 4 dw 分块，
/// 本工具组的 ioctl 里没用 BULK，列出以便对照）
pub const MAX_DATA_SIZE: usize = 100;

// ------------------------------------------------------------ 基础结构体

/// `struct anlogic_ioc_base`
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IocBase {
    pub magic: u32,
    pub command: u32,
}

/// `struct anlogic_ioc_info`
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IocInfo {
    pub base: IocBase,
    pub vendor: u16,
    pub device: u16,
    pub subsystem_vendor: u16,
    pub subsystem_device: u16,
    pub dma_engine_version: u32,
    pub driver_version: u32,
    pub feature_id: u64,
    pub domain: u16,
    pub bus: u8,
    pub dev: u8,
    pub func: u8,
}

/// `struct anlogic_ioc_bar`（单 dw 读写用）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IocBar {
    pub bar_id: u16,
    pub bar_type: u16,
    pub bar_offaddr: u32,
    pub wdata: u32,
    pub rdata: u32,
}

/// `struct cfgwrite`（配置空间读写，本工具组没用到，列出以便对照）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CfgWrite {
    pub reg: i32,
    pub value: i32,
}

// ------------------------------------------------------- 收帧缓冲（DMA）协议

/// `ANLOGIC_FRAME_MAGIC = 0x4D524641u`
pub const ANLOGIC_FRAME_MAGIC: u32 = 0x4D52_4641;
/// `ANLOGIC_FRAME_FLAG_COMPLETE = 0x1u`
pub const ANLOGIC_FRAME_FLAG_COMPLETE: u32 = 0x1;
/// `ANLOGIC_FRAME_MAX_WAIT_MS = 600000u`
pub const ANLOGIC_FRAME_MAX_WAIT_MS: u32 = 600_000;

/// `struct anlogic_frame_mailbox`（64 字节，独占一页）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FrameMailbox {
    pub magic: u32,
    pub seq: u32,
    pub count: u32,
    pub slot_bytes: u32,
    pub slots: u32,
    pub stride: u32,
    pub width: u32,
    pub height: u32,
    /// A 路推帧门铃；B 路门铃紧跟在 +36（见 header 的 push_dbell_b）。
    pub push_dbell: u32,
    pub push_dbell_b: u32,
    pub reserved: [u32; 6],
}

/// `struct anlogic_frame_slot_hdr`（16 字节）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FrameSlotHdr {
    pub seq: u32,
    pub len: u32,
    pub flags: u32,
    pub reserved: u32,
}

/// `struct anlogic_ioc_frame`（ANLOGIC_IOCFRAMEINFO 的出入参）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IocFrame {
    pub phys_addr: u64,
    pub size: u64,
    pub stride: u32,
    pub slot_bytes: u32,
    pub hdr_bytes: u32,
    pub slots: u32,
    pub width: u32,
    pub height: u32,
    pub seq: u32,
    pub count: u32,
    pub mb_off: u32,
}

/// `struct anlogic_ioc_frame_wait`（ANLOGIC_IOCFRAMEWAIT 的出入参）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IocFrameWait {
    pub last_seq: u32,
    pub timeout_ms: u32,
    pub seq: u32,
    pub count: u32,
    pub ready: u32,
    pub slot: u32,
}

// ------------------------------------------------- 相机链路 / 推帧（寄存器层）
//
// 对照头文件里那一组 ANLOGIC_BAR_*_OFF 与 CTRL 位定义（表见
// fpga/rtl/ep_side/img/ep_bar_win.v）。

/// `ANLOGIC_BAR_MAGIC_OFF` / `ANLOGIC_BAR_MAGIC_VAL`
pub const ANLOGIC_BAR_MAGIC_OFF: u32 = 0x40;
pub const ANLOGIC_BAR_MAGIC_VAL: u32 = 0x5043_4946;
/// `ANLOGIC_MMIO_ALL_F`：读到全 F 立即熔断
pub const ANLOGIC_MMIO_ALL_F: u32 = 0xffff_ffff;

pub const ANLOGIC_BAR_CTRL_OFF: u32 = 0x44;
pub const ANLOGIC_BAR_FRAME_CNT_OFF: u32 = 0x4C;
pub const ANLOGIC_BAR_CAM_A_STAT_OFF: u32 = 0x80;
pub const ANLOGIC_BAR_DDR_SEQ_OFF: u32 = 0xA0;
pub const ANLOGIC_BAR_DDR_BASE_OFF: u32 = 0xA4;
pub const ANLOGIC_BAR_DDR_STATUS_OFF: u32 = 0xA8;
pub const ANLOGIC_BAR_PUSH_DONE_OFF: u32 = 0xC0;
pub const ANLOGIC_BAR_PUSH_STATUS_OFF: u32 = 0xC4;
pub const ANLOGIC_BAR_PUSH_ERR_OFF: u32 = 0xC8;
pub const ANLOGIC_BAR_PUSH_TLP_OFF: u32 = 0xCC;

/// `ANLOGIC_CTRL_*`
pub const ANLOGIC_CTRL_ENABLE: u32 = 1 << 0;
pub const ANLOGIC_CTRL_TRIGGER: u32 = 1 << 1;
pub const ANLOGIC_CTRL_MCLK_SEL: u32 = 1 << 2;
/// 旧 RTL 的 CTRL[4]“相机写源”，T07 起保留（写 0/读 0）；不要再用。
pub const ANLOGIC_CTRL_CAM_SEL: u32 = 1 << 4;
/// T07：推帧取哪一路（0 = A，1 = B）
pub const ANLOGIC_CTRL_CAM_B: u32 = 1 << 8;
pub const ANLOGIC_CTRL_DDR_EN: u32 = 1 << 5;
pub const ANLOGIC_CTRL_PUSH_EN: u32 = 1 << 6;
pub const ANLOGIC_CTRL_PUSH_SRC: u32 = 1 << 7;

/// `ANLOGIC_CAM_STAT_LANE_MASK` / `ANLOGIC_CAM_LANE_VALID()` / `ANLOGIC_CAM_LANE_ALL`
pub const ANLOGIC_CAM_STAT_LANE_MASK: u32 = 0x1e;
pub const ANLOGIC_CAM_LANE_ALL: u32 = 0xf;
pub const fn anlogic_cam_lane_valid(stat: u32) -> u32 {
    (stat & ANLOGIC_CAM_STAT_LANE_MASK) >> 1
}

/// `ANLOGIC_DDR_STATUS_*`
pub const ANLOGIC_DDR_STATUS_ALIVE: u32 = 1 << 3;
pub const ANLOGIC_DDR_STATUS_CAL_DONE: u32 = 1 << 4;

/// `ANLOGIC_PUSH_STATUS_*`
pub const ANLOGIC_PUSH_STATUS_BUSY: u32 = 1 << 0;
pub const ANLOGIC_PUSH_STATUS_LAST_OK: u32 = 1 << 1;
pub const ANLOGIC_PUSH_STATUS_DB_SENT: u32 = 1 << 2;

/// `ANLOGIC_PUSH_ERR_*`（0xC8 的取值）
pub const ANLOGIC_PUSH_ERR_NAMES: [&str; 12] = [
    "正常（数据与门铃都发了）",
    "目标地址未配置（整帧没发）",
    "等 tl_tx_wait 超时（本帧停发）",
    "AXI 读响应非 OKAY（本帧停发）",
    "门铃地址未配置（数据推了、门铃没发）",
    "有完整帧但没有 one-shot 额度（未 ARM）",
    "ARM 被拒（描述符/cap/session 校验失败）",
    "CANCEL 取消",
    "HELD 期间源又出帧（已忽略）",
    "源/描述符不一致",
    "活动期写配置被拒",
    "ACK 不匹配（session/seq 与 held 不符）",
];

/// `struct anlogic_ioc_link_status`（ANLOGIC_IOCLINKSTATUS 的出参）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IocLinkStatus {
    pub ctrl: u32,
    pub frame_cnt: u32,
    pub cam_stat: u32,
    pub lane_valid: u32,
    pub cam_locked: u32,
    pub ddr_seq: u32,
    pub ddr_base: u32,
    pub ddr_status: u32,
    pub ddr_ready: u32,
    pub ddr_len: u32,
    pub pushed: u32,
    pub push_status: u32,
    pub push_err: u32,
    pub push_tlp_cnt: u32,
    pub push_en: u32,
    pub push_src: u32,
    pub armed: u32,
    pub dst_lo: u32,
    pub dst_hi: u32,
    pub db_lo: u32,
    pub db_hi: u32,
    pub seq: u32,
    pub count: u32,
}

/// `struct anlogic_ioc_link_retrig`（ANLOGIC_IOCLINKRETRIG 的出入参）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IocLinkRetrig {
    pub wait_ms: u32,
    pub ctrl: u32,
    pub cam_stat: u32,
    pub lane_valid: u32,
    pub cam_locked: u32,
    pub frame_cnt: u32,
    pub frames_flowing: u32,
    pub waited_ms: u32,
}

/// `ANLOGIC_PUSH_CFG_SET_DST` / `ANLOGIC_PUSH_CFG_SET_DB`
pub const ANLOGIC_PUSH_CFG_SET_DST: u32 = 0x1;
pub const ANLOGIC_PUSH_CFG_SET_DB: u32 = 0x2;

/// `struct anlogic_ioc_push_cfg`（ANLOGIC_IOCPUSHCFG 的出入参）
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IocPushCfg {
    pub flags: u32,
    pub dst_lo: u32,
    pub dst_hi: u32,
    pub db_lo: u32,
    pub db_hi: u32,
    pub push_en: u32,
    pub push_src: u32,
    pub armed: u32,
    pub applied: u32,
}

// 全零即「驱动侧还没填」的合法初值；这些都是纯 POD，直接零初始化。
macro_rules! impl_default_zeroed {
    ($($t:ty),* $(,)?) => { $(
        impl Default for $t {
            fn default() -> Self {
                // SAFETY: 都是 #[repr(C)] 的纯 POD（整数/整型数组），全零是合法位模式。
                unsafe { std::mem::zeroed() }
            }
        }
    )* };
}
impl_default_zeroed!(IocBase, IocInfo, IocBar, CfgWrite, FrameMailbox, FrameSlotHdr, IocFrame, IocFrameWait,
                     IocLinkStatus, IocLinkRetrig, IocPushCfg);

// 与头文件一致：sizeof 与关键字段偏移必须钉死（缓冲布局算错就会取到错数据）。
const _: () = {
    assert!(std::mem::size_of::<IocBase>() == 8);
    assert!(std::mem::offset_of!(IocBase, magic) == 0);
    assert!(std::mem::offset_of!(IocBase, command) == 4);
    assert!(std::mem::size_of::<IocInfo>() == 40);
    assert!(std::mem::offset_of!(IocInfo, vendor) == 8);
    assert!(std::mem::offset_of!(IocInfo, device) == 10);
    assert!(std::mem::offset_of!(IocInfo, subsystem_vendor) == 12);
    assert!(std::mem::offset_of!(IocInfo, dma_engine_version) == 16);
    assert!(std::mem::offset_of!(IocInfo, driver_version) == 20);
    assert!(std::mem::offset_of!(IocInfo, feature_id) == 24);
    assert!(std::mem::offset_of!(IocInfo, domain) == 32);
    assert!(std::mem::offset_of!(IocInfo, bus) == 34);
    assert!(std::mem::offset_of!(IocInfo, dev) == 35);
    assert!(std::mem::offset_of!(IocInfo, func) == 36);
    assert!(std::mem::size_of::<IocBar>() == 16);
    assert!(std::mem::offset_of!(IocBar, bar_id) == 0);
    assert!(std::mem::offset_of!(IocBar, bar_type) == 2);
    assert!(std::mem::offset_of!(IocBar, bar_offaddr) == 4);
    assert!(std::mem::offset_of!(IocBar, wdata) == 8);
    assert!(std::mem::offset_of!(IocBar, rdata) == 12);
    assert!(std::mem::size_of::<CfgWrite>() == 8);
    assert!(std::mem::offset_of!(CfgWrite, reg) == 0);
    assert!(std::mem::offset_of!(CfgWrite, value) == 4);
    assert!(std::mem::size_of::<FrameMailbox>() == 64);
    assert!(std::mem::size_of::<FrameSlotHdr>() == 16);
    assert!(std::mem::size_of::<IocFrame>() == 56);
    assert!(std::mem::size_of::<IocFrameWait>() == 24);
    assert!(std::mem::offset_of!(FrameMailbox, seq) == 4);
    assert!(std::mem::offset_of!(FrameMailbox, count) == 8);
    assert!(std::mem::offset_of!(FrameMailbox, stride) == 20);
    assert!(std::mem::offset_of!(FrameMailbox, push_dbell) == 32);
    assert!(std::mem::offset_of!(FrameMailbox, push_dbell_b) == 36);
    assert!(std::mem::offset_of!(FrameSlotHdr, len) == 4);
    assert!(std::mem::offset_of!(FrameSlotHdr, flags) == 8);
    assert!(std::mem::offset_of!(IocFrame, phys_addr) == 0);
    assert!(std::mem::offset_of!(IocFrame, size) == 8);
    assert!(std::mem::offset_of!(IocFrame, stride) == 16);
    assert!(std::mem::offset_of!(IocFrame, hdr_bytes) == 24);
    assert!(std::mem::offset_of!(IocFrame, seq) == 40);
    assert!(std::mem::offset_of!(IocFrame, mb_off) == 48);
    assert!(std::mem::offset_of!(IocFrameWait, last_seq) == 0);
    assert!(std::mem::offset_of!(IocFrameWait, ready) == 16);
    assert!(std::mem::offset_of!(IocFrameWait, slot) == 20);
    assert!(std::mem::size_of::<IocLinkStatus>() == 92);
    assert!(std::mem::offset_of!(IocLinkStatus, cam_stat) == 8);
    assert!(std::mem::offset_of!(IocLinkStatus, pushed) == 40);
    assert!(std::mem::offset_of!(IocLinkStatus, dst_lo) == 68);
    assert!(std::mem::offset_of!(IocLinkStatus, seq) == 84);
    assert!(std::mem::size_of::<IocLinkRetrig>() == 32);
    assert!(std::mem::offset_of!(IocLinkRetrig, lane_valid) == 12);
    assert!(std::mem::offset_of!(IocLinkRetrig, frames_flowing) == 24);
    assert!(std::mem::offset_of!(IocLinkRetrig, waited_ms) == 28);
    assert!(std::mem::size_of::<IocPushCfg>() == 36);
    assert!(std::mem::offset_of!(IocPushCfg, db_lo) == 12);
    assert!(std::mem::offset_of!(IocPushCfg, applied) == 32);
};

// -------------------------------------------------------------- IOCTL 编号
//
// 对照头文件末尾那一段 #define；编号 = _IOC(方向, 'x', 序号, sizeof(结构体))。
// 只列出本工具组用到的三条（NOP/OFFLINE/ONLINE/BULK/CFG 本工具组不碰）。

/// `ANLOGIC_IOCINFO = _IOWR(ANLOGIC_IOC_MAGIC, ANLOGIC_IOC_INFO=1, struct anlogic_ioc_info)`
pub const ANLOGIC_IOCINFO: u64 = ioc(IOC_READ | IOC_WRITE, ANLOGIC_IOC_MAGIC, 1, 40);
/// `ANLOGIC_IOCW = _IOW(ANLOGIC_IOC_MAGIC, 4, struct anlogic_ioc_bar)`
pub const ANLOGIC_IOCW: u64 = ioc(IOC_WRITE, ANLOGIC_IOC_MAGIC, 4, 16);
/// `ANLOGIC_IOCR = _IOR(ANLOGIC_IOC_MAGIC, 5, struct anlogic_ioc_bar)`
pub const ANLOGIC_IOCR: u64 = ioc(IOC_READ, ANLOGIC_IOC_MAGIC, 5, 16);
/// `ANLOGIC_IOCFRAMEINFO = _IOR(ANLOGIC_IOC_MAGIC, 10, struct anlogic_ioc_frame)`
pub const ANLOGIC_IOCFRAMEINFO: u64 = ioc(IOC_READ, ANLOGIC_IOC_MAGIC, 10, 56);
/// `ANLOGIC_IOCFRAMEWAIT = _IOWR(ANLOGIC_IOC_MAGIC, 11, struct anlogic_ioc_frame_wait)`
pub const ANLOGIC_IOCFRAMEWAIT: u64 = ioc(IOC_READ | IOC_WRITE, ANLOGIC_IOC_MAGIC, 11, 24);
/// `ANLOGIC_IOCLINKSTATUS = _IOR(ANLOGIC_IOC_MAGIC, 12, struct anlogic_ioc_link_status)`
pub const ANLOGIC_IOCLINKSTATUS: u64 = ioc(IOC_READ, ANLOGIC_IOC_MAGIC, 12, 92);
/// `ANLOGIC_IOCLINKRETRIG = _IOWR(ANLOGIC_IOC_MAGIC, 13, struct anlogic_ioc_link_retrig)`
pub const ANLOGIC_IOCLINKRETRIG: u64 = ioc(IOC_READ | IOC_WRITE, ANLOGIC_IOC_MAGIC, 13, 32);
/// `ANLOGIC_IOCPUSHCFG = _IOWR(ANLOGIC_IOC_MAGIC, 14, struct anlogic_ioc_push_cfg)`
pub const ANLOGIC_IOCPUSHCFG: u64 = ioc(IOC_READ | IOC_WRITE, ANLOGIC_IOC_MAGIC, 14, 36);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ioctl_numbers_match_header() {
        // 与 gcc 编译 include/anlogic_pci_ioctl.h 打出来的值逐条对齐
        assert_eq!(ANLOGIC_IOCINFO, 0xc028_7801);
        assert_eq!(ANLOGIC_IOCW, 0x4010_7804);
        assert_eq!(ANLOGIC_IOCR, 0x8010_7805);
        assert_eq!(ANLOGIC_IOCFRAMEINFO, 0x8038_780a);
        assert_eq!(ANLOGIC_IOCFRAMEWAIT, 0xc018_780b);
        assert_eq!(ANLOGIC_IOCLINKSTATUS, 0x805c_780c);
        assert_eq!(ANLOGIC_IOCLINKRETRIG, 0xc020_780d);
        assert_eq!(ANLOGIC_IOCPUSHCFG, 0xc024_780e);
    }
}
