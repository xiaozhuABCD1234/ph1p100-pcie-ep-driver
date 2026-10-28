// SPDX-License-Identifier: GPL-2.0
/*!
 * cam_grab —— 从 PH1P100 PCIe EP 的 BAR0 抓一帧 640x480 灰度图，写成 PGM
 * （原 app/cam_grab.c 的 Rust 版，抓帧时序/寄存器语义/输出格式都不变）
 *
 * ⚠ LEGACY：本工具对应 T07 之前“测试图/相机 → fb[] → 0x5C/0x60 PIO”的
 * 640×480 通路。当前 T07 RTL 把 CTRL[4] 保留（写 0/读 0），相机整帧走
 * DDR3 + one-shot 推帧；在当前位流上 -s cam 会因 bit4 回读 0 以 EX_DEV 退出。
 * 因此硬件抓帧默认直接拒绝（不碰 BAR），只有显式 --legacy 才走旧路径；
 * 当前收帧请用 frame_grab（R10F 解包 → PGM）。
 *
 * --------------------------------------------------------------------------
 * 一、寄存器语义（依据 fpga/rtl/ep_side/img/ep_bar_win.v 的读 mux :215-241）
 * --------------------------------------------------------------------------
 *   0x40 MAGIC     只读，期望 0x50434946                   (:216)
 *   0x44 CTRL      R/W bit0 enable / bit1 trigger / [3:2] pattern / bit4 cam_sel
 *                  - 写：ctrl_enable<=bit0、ctrl_trigger<=bit1、pattern<=[3:2]、
 *                    ctrl_cam_sel<=bit4                                (:157-161)
 *                  - trigger 是「写 1 出单帧」的单拍脉冲：没有写 CTRL 的拍会被
 *                    自动清 0（:168-170），主机读回恒为 0
 *                  - 读回布局 {…, cam_sel, pattern, trigger, enable}   (:217)
 *                  注意：写 CTRL 会把 [3:2] 一起覆盖，所以必须先把原值读出来带上。
 *   0x48 STATUS    R   bit0 ready / bit1 busy                        (:218)
 *   0x4C FRAME_CNT R   已完成帧数，写源由 CTRL[4] 决定               (:219)
 *   0x50/0x54      R   宽 / 高 = 640 / 480                           (:220-221)
 *   0x58 DATA_LEN  R   每帧字节数                                    (:222)
 *   0x5C PTR       R/W 帧数据读指针（字节，写 0 复位）               (:223 + :162)
 *   0x60 DATA      R   返回 PTR 处一个 dw，PTR 自动 +4               (:224 + :175-181)
 *   0x64 VERSION / 0x68 CLK_MHZ / 0x6C SCRATCH                       (:225-227)
 *   0x70..0x7C     调试口（PF0/last_wr/fb_raddr/dbg_idx）            (:228-231)
 *   0x80..0x98     cam_stat[31:0] .. cam_stat[223:192]               (:232-238)
 *   0x9C           cam_tap_cfg {manual_en,tap[5:0]}（不是 cam_stat[255:224]）(:239)
 *   未登记 offset 读回 0xDEADBEEF（:240），本工具据此自检「有没有打到别的地址」。
 *   完成包第 0 个 dw 在 [255:224] 且已逐 dw bswap32，所以主机小端读到的就是原值
 *   （:244-248，这也是 MAGIC 能读成 0x50434946 的原因）。
 *
 * --------------------------------------------------------------------------
 * 二、CTRL 的用法（按 RTL 实际语义，不是猜的）
 * --------------------------------------------------------------------------
 *   enable  bit0：只门控「测试图发生器」，对相机写源没有作用
 *                 —— img_frame_src.v:133 `start = (enable | trigger) && (…空闲…)`
 *   trigger bit1：给测试图发生器一个「出一帧」的脉冲；start 只在 !busy_t 时求值
 *                 （img_frame_src.v:150-152），帧中再触发不会插帧
 *   cam_sel bit4：帧缓存写口在「测试图」与「相机」之间二选一（img_frame_src.v:94-96、
 *                 相机写源只接 A 路：ph1p_pcie_ep_top.v:899-917）
 *
 *   本工具的取值：
 *     - 相机   ：CTRL = [3:2] | bit0 | bit4     （相机链路自己连续出帧，不用 trigger）
 *     - 测试图 ：CTRL = [3:2] | bit1           （enable=0：只出一帧，出完就静止）
 *     测试图源故意不用 enable=1（原草稿/实测用的 bit0|bit1）：enable=1 会按
 *     IDLE_CYCLE=1000000 连续重写整块帧缓存（ph1p_pcie_ep_top.v:591-595），
 *     在 62MHz 下约 16ms 一帧、整帧搬运要 100ms 以上 —— 边写边读必然撕裂。
 *     改成单帧脉冲后帧缓存静止，抓到的是一帧完整数据（抓完再核对帧号证明没动）。
 *
 * --------------------------------------------------------------------------
 * 三、帧缓存布局（决定了 PGM 怎么摆）
 * --------------------------------------------------------------------------
 *   字 0..76799 = 像素，4 像素/32bit 字、小端（x 小的在低字节，
 *                 img_frame_src.v:12-16、cam_fb_writer.v:45-52）
 *   ⇒ 像素 k 在字 (k/4) 的第 (k%4) 字节；一帧 640x480 = 307200 像素，
 *     帧缓存 76800 字（307200 字节，ep_bar_win.v:46 PTR_BYTES=0x4B000）
 *     正好装下整帧，第 0 行最左 4 个像素也在缓存里。
 *   帧号不占帧缓存：它就是寄存器 0x4C FRAME_CNT（相机源在 EOF 递增，
 *   cam_fb_writer.v:286；测试图源在帧末递增，img_frame_src.v:210），
 *   所以本工具只从 0x4C 读帧号，不再去缓存里对账。
 *
 * --------------------------------------------------------------------------
 * 四、相机关态字（0x80 = A 路，0x84 = B 路）逐位含义
 * --------------------------------------------------------------------------
 *   位定义出处 ph1p_pcie_ep_top.v:1036-1038（w_cam_a_stat）：
 *     bit0      字节时钟在跳（clk_activity 看到 w_a_byte_clk 翻过）
 *     [4:1]     lane_valid[3:0]，mipi_csi2_rx_ph1p.v:32-33/:57 每 lane 字节对齐有效
 *     bit5      I2C 配置完成 cfg_done（ui_cfg_imx415）
 *     bit6      I2C 总线错误 bus_err
 *     bit7      auto_delay_done
 *     bit8      in_frame（电平） / bit9 in_line（行首脉冲）/ bit10 收到过像素组
 *     [21:16]   当前 IDELAY tap
 *     bit22     xclr/r_rst_n_slow 已放开 / bit23 csi_rst_n
 *     bit24/25  SCL / SDA 电平
 *     [31:26]   固定标签 6'b110011（用来确认读到的确实是状态字而不是垃圾）
 *   拆位顺序依据 ph1p_pcie_ep_top.v:1042-1044 的拼接：
 *     cam_stat = {32'd0, {r_pwr_cnt2,8'd0,w_a_i2c_busy}, r_b_pix, r_a_pix,
 *                 r_b_frames, r_a_frames, w_cam_b_stat, w_cam_a_stat}
 *     ⇒ 0x80=A 状态、0x84=B 状态、0x88=A 路 VSYNC 计数、0x90=A 路像素计数、
 *       0x98={r_pwr_cnt2,8'd0,w_a_i2c_busy}（bit0=I2C 忙）
 *
 * --------------------------------------------------------------------------
 * 五、访问规约（逐条落实）
 * --------------------------------------------------------------------------
 * 本工具刻意不走驱动的 ioctl，而是直接 mmap sysfs 的 resource0：PIO 抓帧对每次
 * 寄存器访问的开销敏感，走 ioctl 要多付进程切换与驱动的串行化锁。pci-sysfs 的
 * mmap（内核实现在 drivers/pci/pci-sysfs.c）不检查资源是否已被驱动 claim，因此
 * 三条门禁——先读 0x40 验 MAGIC、单次不超过 4 个 dw、读到 0xffffffff 立即熔断
 * ——由本工具自持（magic_gate / Regs::rd / Regs::data_rd），不依赖驱动侧校验。
 *
 *   A-1 :201 只碰寄存器表登记过的 offset → off_rd_ok()/off_wr_ok() 白名单，
 *             DATA 口 0x60 只能经 data_rd() 访问（读了就动 PTR，不能随手读）
 *   A-3 :204 不做扫窗枚举 → 只读 0x40 判存在性
 *   B-4 :207 每次读之前先读 0x40 MAGIC，不符即停 → magic_gate()，抓帧前/后各一次
 *   B-5 :208 任何一次读拿到 0xffffffff 立即停止 → reg_rd()/data_rd() 命中即熔断退出
 *   C-7 :213 单次 ≤ 4 dw，且只用 32 位对齐的 volatile u32 逐 dw 读 → 本工具所有
 *             访问都是单 dw（4 字节）32 位对齐，从不做 16 字节长读
 *   C-8 :215 不用 memcpy_fromio/ioread32_rep/宽访问 → mmap 上只有 volatile u32 读
 *   C-9 :217 只对 0x60 读数据、遵守「读后 PTR 自动 +4、同地址不同值」
 *   D-14:230 mmap 只用 offset=0、length ≤ 4KB、只按 4B 访问 → 见 BAR_WIN_BYTES
 *   E-16:235 轮询间隔 ≥ 1ms → POLL_MS
 *   E-17:236 一次测试一个进程持有设备 → flock(LOCK_EX|LOCK_NB)
 *   另外在发任何 MMIO 之前先用 sysfs 的 vendor/device 确认设备身份（期望
 *   0x1edb:0xabcd，出处 fpga/al_ip/pcie_ep_core/.params_pcie_ep_core.txt:55-56），
 *   免得默认路径在别的机器上指向另一块设备时被误读寄存器。
 *   另外：0x60 的预取是 4 拍轮转（ep_bar_win.v:186-203，注释「主机访问间隔远大于此」），
 *   一次 PCIe MRd 往返 ≫ 4 拍，所以逐 dw 读安全。
 *
 * --------------------------------------------------------------------------
 * 六、退出码
 * --------------------------------------------------------------------------
 *   0 成功（图已写出，一致性校验通过）
 *   1 命令行用法错误
 *   2 设备不可用：open/mmap/flock 失败、MAGIC 不符、读到 0xffffffff、几何不符
 *   3 相机链路未就绪（字节时钟/lane_valid/cfg_done）—— 不写图
 *   4 超时等不到新帧 —— 不写图
 *   5 图已写出，但一致性校验不过（帧号对不上 / 搬运时帧号变过 / 写口一直忙 / 测试图自检不符）
 *   6 输出文件或内存错误
 *
 * 用法：cam_grab [-s cam|pattern] [-d /sys/…/resource0] [-t 超时ms] [输出.pgm]
 *       cam_grab --decode 0xcdee0020      # 只把状态字解码成人话，不碰硬件
 */
use std::cell::Cell;
use std::io::Write;
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::time::{Duration, Instant};

use anlogic_pci_tools::getopt::{Getopt, LongOpt};
use anlogic_pci_tools::parse::strtoul_base0;
use anlogic_pci_tools::sys::{self, Mmap, PROT_READ, PROT_WRITE};

// ---------------------------------------------------------------- 常量

const DEF_DEV: &str = "/sys/bus/pci/devices/0000:01:00.0/resource0";
const DEF_OUT: &str = "frame.pgm";
const DEF_TIMEOUT_MS: u32 = 3000;

const BAR_WIN_BYTES: usize = 4096; // mmap 只取 4KB（帧数据走 0x60 口搬运）
const MAGIC_EXPECT: u32 = 0x5043_4946; // ep_bar_win.v:216 + 参数 :43
const VAL_ALLF: u32 = 0xffff_ffff;
const VAL_DEADBEEF: u32 = 0xdead_beef; // ep_bar_win.v:240 default 分支

const EP_VENDOR_ID: u32 = 0x1edb; // fpga/al_ip/pcie_ep_core/.params_pcie_ep_core.txt:55
const EP_DEVICE_ID: u32 = 0xabcd; // 同上 :56（与厂商驱动匹配表一致）

const FB_BYTES: usize = 0x4B000; // ep_bar_win.v:46 PTR_BYTES（640*480）
const FB_WORDS: usize = FB_BYTES / 4; // 76800 字
const FB_PIXELS: usize = FB_WORDS * 4; // 字 0..76799 → 307200 个像素（整帧）
const FB_IMAGE_PIX: u32 = 640 * 480; // 期望的 W*H

// 寄存器表：只有 ep_bar_win.v 读 mux（:216-239）登记过的 offset 才允许访问
const R_MAGIC: u32 = 0x40; // :216
const R_CTRL: u32 = 0x44; // :217 / 写语义 :157-161 / trigger 自动清 :168-170
const R_STATUS: u32 = 0x48; // :218 {…, busy, ready}
const R_FRAME: u32 = 0x4C; // :219
const R_WIDTH: u32 = 0x50; // :220
const R_HEIGHT: u32 = 0x54; // :221
const R_LEN: u32 = 0x58; // :222
const R_PTR: u32 = 0x5C; // :223 / 写 :162
const R_DATA: u32 = 0x60; // :224，读后 PTR += 4（:111/:152/:175-181）
const R_VERSION: u32 = 0x64; // :225
const R_CLK_MHZ: u32 = 0x68; // :226
const R_CAM_A: u32 = 0x80; // :232 cam_stat[31:0]  = A 路状态
const R_CAM_B: u32 = 0x84; // :233 cam_stat[63:32] = B 路状态
const R_CAM_A_FRAMES: u32 = 0x88; // :234 A 路 VSYNC 计数
const R_CAM_B_FRAMES: u32 = 0x8C; // :235 B 路 VSYNC 计数
const R_CAM_A_PIX: u32 = 0x90; // :236 A 路像素组计数
const R_CAM_B_PIX: u32 = 0x94; // :237 B 路像素组计数
const R_CAM_I2C: u32 = 0x98; // :238 {r_pwr_cnt2,8'd0,i2c_busy}
const R_TAP: u32 = 0x9C; // :239 cam_tap_cfg {manual_en,tap[5:0]}

const CTRL_ENABLE: u32 = 1 << 0;
const CTRL_TRIGGER: u32 = 1 << 1;
const CTRL_PAT_SHIFT: u32 = 2;
const CTRL_PAT_MASK: u32 = 3 << CTRL_PAT_SHIFT;
const CTRL_CAM_SEL: u32 = 1 << 4;

const STATUS_READY: u32 = 1 << 0;
const STATUS_BUSY: u32 = 1 << 1;

const POLL_MS: u32 = 10; // 轮询间隔 ≥ 1ms
const RETRIGGER_MS: u32 = 100; // 测试图源重发 trigger 的最小间隔（IDLE_CYCLE≈16ms@62MHz）

// 退出码
const EX_OK: i32 = 0;
const EX_USAGE: i32 = 1;
const EX_DEV: i32 = 2;
const EX_CAM_NOT_READY: i32 = 3;
const EX_NO_FRAME: i32 = 4;
const EX_INCONSISTENT: i32 = 5;
const EX_OUTPUT: i32 = 6;

/// 只映射 BAR0 前 4KB 的窗口（只做 32 位对齐的单 dw 访问）
struct Regs {
    ptr: *mut u32,
    /// 读到 0xffffffff 的次数（命中即熔断，正常恒 0）
    allf: Cell<u32>,
}

/// 允许「读」的 offset（0x60 除外：读它就会动 PTR，只能走 data_rd）
fn off_rd_ok(off: u32) -> bool {
    [
        R_MAGIC, R_CTRL, R_STATUS, R_FRAME, R_WIDTH, R_HEIGHT, R_LEN, R_PTR, R_VERSION, R_CLK_MHZ,
        0x6C, 0x70, 0x74, 0x78, 0x7C, R_CAM_A, R_CAM_B, R_CAM_A_FRAMES, R_CAM_B_FRAMES, R_CAM_A_PIX,
        R_CAM_B_PIX, R_CAM_I2C, R_TAP,
    ]
    .contains(&off)
}

/// 允许「写」的 offset：只用得到 CTRL 与 PTR（ep_bar_win.v:157-165）
fn off_wr_ok(off: u32) -> bool {
    [R_CTRL, R_PTR].contains(&off)
}

impl Regs {
    /// 读一个登记过的寄存器：32 位对齐、单 dw。
    /// 0xffffffff → 熔断；0xDEADBEEF → 说明打到了未译码地址（:240）。
    fn rd(&self, off: u32) -> Result<u32, i32> {
        if !off_rd_ok(off) {
            eprintln!("!! 程序内部错误：试图读未登记的 offset 0x{:02x}", off);
            return Err(EX_DEV);
        }
        // SAFETY: off 在 4KB 窗口内（白名单已挡），且窗口是 4 字节对齐的 mmap
        let v = unsafe { sys::rd32(self.ptr.add((off >> 2) as usize)) };
        if v == VAL_ALLF {
            self.allf.set(self.allf.get() + 1);
            eprintln!(
                "!! 读 0x{:02x} 得到 0xffffffff：设备已不可用，立即停止，不再发任何 MMIO",
                off
            );
            return Err(EX_DEV);
        }
        if v == VAL_DEADBEEF {
            eprintln!(
                "!! 读 0x{:02x} 得到 0xdeadbeef：FPGA 回的是未译码默认值（ep_bar_win.v:240），该 offset 不该被访问",
                off
            );
            return Err(EX_DEV);
        }
        Ok(v)
    }

    fn wr(&self, off: u32, v: u32) -> Result<(), i32> {
        if !off_wr_ok(off) {
            eprintln!("!! 程序内部错误：试图写未登记的 offset 0x{:02x}", off);
            return Err(EX_DEV);
        }
        // SAFETY: off 在 4KB 窗口内（白名单已挡），且窗口是 4 字节对齐的 mmap
        unsafe { sys::wr32(self.ptr.add((off >> 2) as usize), v) };
        Ok(())
    }

    /// 读 0x60 数据口：返回当前 PTR 处一个 dw，PTR 自动 +4（ep_bar_win.v:111/:152/:175-181）。
    /// 只用单 dw：① 单次访问不得超过 4 个 dw；② 预取 4 拍轮转是给「访问间隔远大于 4 拍」
    /// 设计的（ep_bar_win.v:186-203），一次 MRd 往返远大于此；③ 同地址连续读不保证同值。
    fn data_rd(&self, idx: usize) -> Result<u32, i32> {
        // SAFETY: R_DATA 固定落在 4KB 窗口里
        let v = unsafe { sys::rd32(self.ptr.add((R_DATA >> 2) as usize)) };
        if v == VAL_ALLF {
            self.allf.set(self.allf.get() + 1);
            eprintln!("!! 读 0x60 数据口第 {} 个字得到 0xffffffff：设备已不可用，立即停止", idx);
            return Err(EX_DEV);
        }
        Ok(v)
    }

    /// 任何一轮访问之前先自检 MAGIC。
    /// 读 0x40 不会动 PTR（只有 0x60 会），所以抓帧前后各查一次是安全的。
    fn magic_gate(&self, when: &str) -> i32 {
        let m = match self.rd(R_MAGIC) {
            Ok(v) => v,
            Err(rc) => return rc,
        };
        if m != MAGIC_EXPECT {
            eprintln!(
                "!! MAGIC=0x{:08x} ≠ 0x{:08x}（{}）：设备未就绪或链路已断，不继续碰任何寄存器、不写图",
                m, MAGIC_EXPECT, when
            );
            return EX_DEV;
        }
        EX_OK
    }
}

// ---------------------------------------------------------------- 小工具

/// `clock_gettime(CLOCK_MONOTONIC)` 的等价（进程内单调毫秒）
fn now_ms() -> u64 {
    static T0: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    T0.get_or_init(Instant::now).elapsed().as_millis() as u64
}

fn msleep(ms: u32) {
    std::thread::sleep(Duration::from_millis(ms as u64));
}

// ------------------------------------------------- 相机状态字解码 / 判据

/// 逐位打印（位定义 ph1p_pcie_ep_top.v:1036-1038），out 可以是 stdout 或 stderr
fn cam_stat_decode(s: u32, tag: &str, out: &mut dyn Write) {
    let bclk = (s >> 0) & 1;
    let lane = (s >> 1) & 0xf;
    let cfg = (s >> 5) & 1;
    let buse = (s >> 6) & 1;
    let adone = (s >> 7) & 1;
    let infr = (s >> 8) & 1;
    let inln = (s >> 9) & 1;
    let pv = (s >> 10) & 1;
    let rsvd = (s >> 11) & 0x1f;
    let tap = (s >> 16) & 0x3f;
    let slow = (s >> 22) & 1;
    let rstn = (s >> 23) & 1;
    let scl = (s >> 24) & 1;
    let sda = (s >> 25) & 1;
    let tag6 = (s >> 26) & 0x3f;

    let _ = writeln!(
        out,
        "  {} 状态字 0x{:08x} 逐位解码（位定义 ph1p_pcie_ep_top.v:1036-1038）:",
        tag, s
    );
    let _ = writeln!(
        out,
        "    bit0       字节时钟在跳            = {}{}",
        bclk,
        if bclk != 0 { "" } else { "   ← 软 PHY 字节时钟没有活动，MIPI 高速时钟没进来" }
    );
    let _ = writeln!(
        out,
        "    bit[4:1]   lane_valid[3:0]         = 0x{:x}{}",
        lane,
        if lane == 0xf {
            "（4 条 lane 全对齐）"
        } else if lane == 0 {
            "   ← 没有任何一条 lane 字节对齐，MIPI 链路未锁定"
        } else {
            "   ← 只有部分 lane 对齐"
        }
    );
    let _ = writeln!(
        out,
        "    bit5       I2C 配置完成 cfg_done   = {}{}",
        cfg,
        if cfg != 0 { "" } else { "   ← IMX415 寄存器还没配置完" }
    );
    let _ = writeln!(out, "    bit6       I2C 总线错误 bus_err    = {}", buse);
    let _ = writeln!(out, "    bit7       auto_delay_done         = {}", adone);
    let _ = writeln!(out, "    bit8       in_frame（帧内电平）    = {}", infr);
    let _ = writeln!(out, "    bit9       in_line（行首脉冲）     = {}", inln);
    let _ = writeln!(out, "    bit10      收到过像素组 pv_seen    = {}", pv);
    let _ = writeln!(out, "    bit[15:11] 保留                    = 0x{:x}", rsvd);
    let _ = writeln!(out, "    bit[21:16] 当前 IDELAY tap         = {}", tap);
    let _ = writeln!(out, "    bit22      xclr/r_rst_n_slow 已放开= {}", slow);
    let _ = writeln!(out, "    bit23      csi_rst_n               = {}", rstn);
    let _ = writeln!(out, "    bit24/25   SCL / SDA 电平          = {} / {}", scl, sda);
    let _ = writeln!(
        out,
        "    bit[31:26] 固定标签                = 0x{:02x}（期望 0x33）",
        tag6
    );
}

/// 判据：返回 EX_OK = 可以继续抓帧；否则返回 EX_CAM_NOT_READY（已打印明确原因）。
/// 必死项按任务要求：字节时钟没跳 / lane_valid==0 / cfg_done==0；
/// bus_err、部分 lane、没收到过像素组只告警（帧号与一致性校验还会兜底）。
fn cam_stat_verdict(s: u32, tag: &str, out: &mut dyn Write) -> i32 {
    let bclk = (s >> 0) & 1;
    let lane = (s >> 1) & 0xf;
    let cfg = (s >> 5) & 1;
    let buse = (s >> 6) & 1;
    let pv = (s >> 10) & 1;
    let mut fatal = false;

    if bclk == 0 {
        let _ = writeln!(
            out,
            "!! {} 链路未就绪：字节时钟没有在跳（bit0=0）→ 软 PHY 收不到 MIPI 高速时钟，后面无论等多久都不会有帧",
            tag
        );
        fatal = true;
    }
    if lane == 0 {
        let _ = writeln!(
            out,
            "!! {} 链路未就绪：lane_valid=0（bit[4:1]=0）→ 4 条 lane 全部字节未对齐，MIPI 链路没有锁（PHY/接线/摄像头供电/参考时钟）",
            tag
        );
        fatal = true;
    }
    if cfg == 0 {
        let _ = writeln!(
            out,
            "!! {} 链路未就绪：cfg_done=0（bit5=0）→ IMX415 的 I2C 寄存器还没配置完，帧不可能出来",
            tag
        );
        fatal = true;
    }
    if buse != 0 {
        let _ = writeln!(out, "!! {} 告警：I2C bus_err=1（bit6）→ 传感器配置过程出过总线错误", tag);
    }
    if lane != 0 && lane != 0xf {
        let _ = writeln!(
            out,
            "!! {} 告警：只有部分 lane 对齐（lane_valid=0x{:x}，期望 0xf）",
            tag, lane
        );
    }
    if pv == 0 {
        let _ = writeln!(out, "!! {} 告警：还没收到过任何像素组（bit10=0）", tag);
    }

    if fatal {
        EX_CAM_NOT_READY
    } else {
        EX_OK
    }
}

// ------------------------------------------------------- 设备身份门禁

/// fscanf("%x") 读一个 sysfs 里的十六进制数
fn read_sysfs_hex(dir: &str, name: &str) -> Option<u32> {
    let s = std::fs::read_to_string(format!("{}/{}", dir, name)).ok()?;
    anlogic_pci_tools::parse::hex_prefix_u32(&s)
}

/// 发任何 MMIO 之前先确认「这确实是 PH1P100 EP」：0x40/0x44 这种读在别的设备上
/// 是「读它的私有寄存器」，必须避免。只读 sysfs，不碰设备本身。
/// 读不到 vendor/device（不是 sysfs 路径）时只告警放行，由使用者负责。
fn device_id_gate(dev_path: &str) -> i32 {
    let slash = match dev_path.rfind('/') {
        Some(p) if p != 0 && dev_path.len() < 256 => p,
        _ => {
            eprintln!(
                "!! 警告：{} 不像 sysfs 里的 resourceN，跳过设备身份自检\n   （请自行确认它确实是 PH1P100 EP 的 BAR0 映射）",
                dev_path
            );
            return EX_OK;
        }
    };
    let dir = &dev_path[..slash];

    let (vid, did) = match (read_sysfs_hex(dir, "vendor"), read_sysfs_hex(dir, "device")) {
        (Some(v), Some(d)) => (v, d),
        _ => {
            eprintln!("!! 警告：读不到 {}/vendor|device，跳过设备身份自检", dir);
            return EX_OK;
        }
    };
    if vid != EP_VENDOR_ID || did != EP_DEVICE_ID {
        eprintln!(
            "!! {} 的 PCI ID 是 {:04x}:{:04x}，不是 PH1P100 EP 的 {:04x}:{:04x}\n   → 拒绝在它上面发任何 MMIO（默认路径是给板子的，别的机器上\n   0000:01:00.0 往往是另一块设备）。出处：fpga/al_ip/pcie_ep_core/\n.params_pcie_ep_core.txt:55-56（PCIE_CFG_PF0_*）",
            dev_path, vid, did, EP_VENDOR_ID, EP_DEVICE_ID
        );
        return EX_DEV;
    }
    println!("  设备身份：{:04x}:{:04x}（PH1P100 EP ✓）", vid, did);
    EX_OK
}

// ------------------------------------------------------------- 帧就绪等待

/// 等 0x4C 帧号相对 f0 变化（并且 busy 落回 0）。
/// retrigger=true 时每 RETRIGGER_MS 重发一次 trigger：
///   img_frame_src.v:133 的 start 被 (ready_t==0 || idle>=IDLE_CYCLE) 卡着，
///   IDLE_CYCLE=1000000（ph1p_pcie_ep_top.v:594）≈16ms@62MHz，上一帧刚出完时
///   第一发 trigger 可能被吞掉；帧中重发是安全的：start 只在 !busy_t 时求值
///   （img_frame_src.v:150-152），不会插帧。
fn wait_new_frame(
    regs: &Regs,
    f0: u32,
    timeout_ms: u32,
    retrigger: bool,
    ctrl_base: u32,
    tag: &str,
) -> i32 {
    let t0 = now_ms();
    let mut last_pulse = t0;

    loop {
        let f = match regs.rd(R_FRAME) {
            Ok(v) => v,
            Err(rc) => return rc,
        };
        if f != f0 {
            let st = match regs.rd(R_STATUS) {
                Ok(v) => v,
                Err(rc) => return rc,
            };
            if st & STATUS_BUSY == 0 {
                println!("  {}：帧号 {} → {}，busy=0，一帧写完（{} ms）", tag, f0, f, now_ms() - t0);
                return EX_OK;
            }
        }
        if now_ms() - t0 >= timeout_ms as u64 {
            eprintln!("!! {}：等了 {} ms 帧号仍是 {}（起等值 {}），没有新帧", tag, timeout_ms, f, f0);
            if let Ok(stl) = regs.rd(R_STATUS) {
                eprintln!(
                    "   最后 STATUS=0x{:08x}（bit0 ready={}, bit1 busy={}）",
                    stl,
                    stl & STATUS_READY,
                    (stl >> 1) & 1
                );
            }
            if let Ok(fl) = regs.rd(R_FRAME) {
                if fl != f {
                    eprintln!("   再读 0x4C = {}（期间又变了，说明帧在出但节奏很慢）", fl);
                }
            }
            return EX_NO_FRAME;
        }
        if retrigger && now_ms() - last_pulse >= RETRIGGER_MS as u64 {
            if let Err(rc) = regs.wr(R_CTRL, ctrl_base | CTRL_TRIGGER) {
                return rc;
            }
            last_pulse = now_ms();
        }
        msleep(POLL_MS);
    }
}

// ------------------------------------------------- 帧缓存 → PGM 像素映射

/// 像素 k ← 帧缓存字 (k/4) 的第 (k%4) 字节，低字节 = 最左像素：
///   - 字 0..76799 全是像素（帧号不占缓存，它是 0x4C/0xA0 这些计数器寄存器）
///   - 4 像素/32bit 字、小端（img_frame_src.v:12-16；相机同布局 cam_fb_writer.v:125）
///   - 帧缓存 76800 字 = 307200 字节，正好装下 640x480 整帧
/// 剩下的字节保持 0，由调用者先清零。
fn fb_to_image(fb: &[u32], img: &mut [u8], npix_store: usize) {
    let n = std::cmp::min(npix_store, img.len());
    for k in 0..n {
        let w = fb[k / 4];
        img[k] = ((w >> (8 * (k % 4) as u32)) & 0xff) as u8;
    }
}

// --------------------------------------------------------- 测试图源自检

/// 测试图源的像素公式（img_frame_src.v:15-17）：
///   pix = x[7:0] + y[7:0] + frame_cnt[7:0] + PIX_ADD，顶层没覆盖 PIX_ADD（默认 0）
///   （ph1p_pcie_ep_top.v:591-595 只传了 W/H/IDLE_CYCLE）
/// 逐像素核对，既验证搬运正确性，也验证「字 0..76799 = 像素（字 k 装 x..x+3）」
/// 这个布局理解（帧号不占缓存，由调用者另外从 0x4C 给）。
fn pattern_check(img: &[u8], n: usize, w: usize, frame: u32, what: &str) -> usize {
    let mut bad = 0usize;

    for k in 0..n {
        let x = (k % w) as u32;
        let y = (k / w) as u32;
        let want = ((x + y + (frame & 0xff)) & 0xff) as u8;
        if img[k] != want {
            if bad < 4 {
                eprintln!(
                    "!! 测试图自检不符：像素 {}（x={} y={}）期望 0x{:02x} 实际 0x{:02x}",
                    k, x, y, want, img[k]
                );
            }
            bad += 1;
        }
    }
    print!("  模式自检        : {}/{} 像素与 (x+y+帧号)&0xff 一致", n - bad, n);
    if bad != 0 {
        print!("  ← {} 个不符（{}）", bad, what);
    }
    println!();
    bad
}

/// ---- --selftest：主机侧「整块缓存都是像素」映射与自检逻辑的离线自证 ----
/// 合成一帧（按 RTL 公式生成帧缓存内容）喂给本工具真正用的那两段代码：
///   ① 本工具的映射必须 0 处不符；② 旧版那种「字 0 当帧号、像素从字 1 起」的
/// 错位映射必须被查出不符。不访问任何硬件。
fn self_test() -> i32 {
    let w = 640usize;
    let h = 480usize;
    let npix_img = w * h;
    let npix_store = FB_PIXELS;
    let frame = 7u32;

    println!("=== --selftest：主机侧像素映射/自检逻辑离线自证（未访问硬件）===");
    println!(
        "  合成帧：字 0..{} 全是像素 (x+y+帧号)&0xff（帧号 {} 只在寄存器里）",
        FB_WORDS - 1,
        frame
    );

    let mut fb = vec![0u32; FB_WORDS];
    for k in 0..npix_store {
        let px = (k % w) as u32;
        let py = (k / w) as u32;
        let p = (px + py + (frame & 0xff)) & 0xff;
        fb[k / 4] |= p << (8 * (k % 4) as u32);
    }

    let mut ret = EX_OK;

    let mut img = vec![0u8; npix_img];
    fb_to_image(&fb, &mut img, npix_store);
    if pattern_check(&img, npix_store, w, frame, "合成数据本不该不符") != 0 {
        eprintln!("!! 自证失败：本工具的映射/自检逻辑对合成帧报不符");
        ret = EX_INCONSISTENT;
    }

    // 反证：旧版映射（字 0 当帧号、像素从字 1 起）→ 必须被抓出来
    let mut img2 = vec![0u8; npix_img];
    for k in 0..npix_store {
        let wv = fb.get(k / 4 + 1).copied().unwrap_or(0);
        img2[k] = ((wv >> (8 * (k % 4) as u32)) & 0xff) as u8;
    }
    if pattern_check(&img2, npix_store, w, frame, "这是旧映射，本就该不符") == 0 {
        eprintln!("!! 自证失败：自检没能查出已知的 4 像素错位");
        ret = EX_INCONSISTENT;
    }

    println!(
        "  结论：映射与自检逻辑自证{}",
        if ret == EX_OK { "通过" } else { "**失败**" }
    );
    ret
}

// ------------------------------------------------------------- 使用说明

fn usage(prog: &str) {
    println!(
        "用法: {} [选项] [输出.pgm]\n\
         \n\
         从 PH1P100 PCIe EP 的 BAR0 抓一帧 640x480 灰度图（LEGACY；当前位流默认拒绝，\n\
         需要 --legacy 才走旧路径）。\n\
         访问规矩：先读 0x40 MAGIC 自检、只碰登记过的\n\
         offset、单次单 dw、任一读拿到 0xffffffff 立即熔断退出。\n\
         \n\
         选项:\n\
         \x20 -s, --source cam|pattern   写源（默认 cam；对应 CTRL[4]=1/0）\n\
         \x20 -d, --dev PATH             BAR0 资源节点（默认 {}）\n\
         \x20 -t, --timeout MS           等新帧超时（默认 {}，最小 50）\n\
         \x20     --decode 0xXXXXXXXX    只把相机状态字逐位解码成人话，不碰任何硬件\n\
         \x20                            （可给多个；0x80=A 路、0x84=B 路）\n\
         \x20     --selftest             离线自证主机侧像素映射/自检逻辑（合成帧，不碰硬件）\n\
         \x20     --legacy               强制走 T07 之前的 640x480 PIO 旧路径（当前位流不要用）\n\
         \x20 -h, --help                 本说明\n\
         \n\
         测试图源是「单帧触发」：CTRL=[3:2]|bit1（enable=0，出一帧就静止），\n\
         所以帧缓存不会被边写边读地撕裂。相机源由链路连续出帧，搬运期间若帧号\n\
         变化（或写口一直忙）会判为可疑：图仍写出，但退出码 5 + 告警，不当成功。\n\
         \n\
         退出码: 0 成功 / 1 用法 / 2 设备不可用 / 3 相机链路未就绪（不写图）/\n\
         \x20\x20\x20\x20\x20\x20\x20\x204 等不到新帧（不写图）/ 5 图已写出但校验不过 / 6 输出文件错误",
        prog, DEF_DEV, DEF_TIMEOUT_MS
    );
}

fn parse_source(s: &str) -> Option<u32> {
    match s {
        "cam" | "camera" | "1" => Some(1),
        "pattern" | "test" | "0" => Some(0),
        _ => None,
    }
}

// ----------------------------------------------------------------- main

fn run() -> i32 {
    let argv: Vec<String> = std::env::args_os()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();

    let longopts = [
        LongOpt { name: "source", has_arg: true, val: b's' as u32 },
        LongOpt { name: "dev", has_arg: true, val: b'd' as u32 },
        LongOpt { name: "timeout", has_arg: true, val: b't' as u32 },
        LongOpt { name: "decode", has_arg: true, val: 1 },
        LongOpt { name: "selftest", has_arg: false, val: 2 },
        LongOpt { name: "legacy", has_arg: false, val: 3 },
        LongOpt { name: "help", has_arg: false, val: b'h' as u32 },
    ];

    let mut getopt = Getopt::new(&argv, "s:d:t:h", &longopts);

    let mut dev_path = DEF_DEV.to_string();
    let mut out_path: Option<String> = None;
    let mut src_cam = true;
    let mut timeout_ms: u32 = DEF_TIMEOUT_MS;
    let mut decode_vals: Vec<u32> = Vec::new();
    let mut want_selftest = false;
    let mut legacy = false;

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
            b's' => {
                let s = arg.unwrap_or_default();
                match parse_source(&s) {
                    Some(v) => src_cam = v != 0,
                    None => {
                        eprintln!("!! --source 只接受 cam 或 pattern（收到 '{}'）", s);
                        return EX_USAGE;
                    }
                }
            }
            b'd' => dev_path = arg.unwrap_or_default(),
            b't' => {
                let s = arg.unwrap_or_default();
                match strtoul_base0(&s) {
                    Some(v) if v != 0 && v <= 600_000 => timeout_ms = v as u32,
                    _ => {
                        eprintln!("!! --timeout 需要 1..600000 的毫秒数（收到 '{}'）", s);
                        return EX_USAGE;
                    }
                }
            }
            1 => {
                let s = arg.unwrap_or_default();
                if decode_vals.len() >= 16 {
                    eprintln!("!! --decode 最多 16 个值");
                    return EX_USAGE;
                }
                match strtoul_base0(&s) {
                    Some(v) if v <= 0xffff_ffff => decode_vals.push(v as u32),
                    _ => {
                        eprintln!("!! --decode 需要 32bit 十六进制值（收到 '{}'）", s);
                        return EX_USAGE;
                    }
                }
            }
            b'h' => {
                usage(&argv[0]);
                return EX_OK;
            }
            2 => want_selftest = true,
            3 => legacy = true,
            _ => {
                usage(&argv[0]);
                return EX_USAGE;
            }
        }
    }

    // 位置参数：输出文件名；兼容旧写法 `cam_grab out.pgm pattern`
    for a in getopt.positional() {
        if (a == "cam" || a == "pattern") && parse_source(&a).is_some() {
            src_cam = parse_source(&a).unwrap() != 0;
        } else if out_path.is_none() {
            out_path = Some(a);
        } else {
            eprintln!("!! 多余的参数：'{}'", a);
            return EX_USAGE;
        }
    }
    let out_path = out_path.unwrap_or_else(|| DEF_OUT.to_string());
    if timeout_ms < 50 {
        timeout_ms = 50;
    }

    // ---- --decode / --selftest：纯离线，不打开设备、不做任何 MMIO ----
    if want_selftest {
        return self_test();
    }
    if !decode_vals.is_empty() {
        println!("=== --decode 离线解码（未访问任何硬件）===");
        for (i, v) in decode_vals.iter().enumerate() {
            println!("--- 状态字 #{}（若来自 0x80 则是 A 路，0x84 则是 B 路）", i + 1);
            cam_stat_decode(*v, "相机", &mut std::io::stdout());
            let _ = cam_stat_verdict(*v, "相机", &mut std::io::stdout());
            println!();
        }
        return EX_OK;
    }

    /*
     * 当前 RTL 已经没有 640x480 fb[] → PIO 这条路（CTRL[4] 保留、读回 0），
     * 默认直接拒绝，免得用户以为工具坏了或者把链路写乱。老位流调试才加
     * --legacy 强制走下面的旧路径。
     */
    if !legacy {
        eprintln!("!! cam_grab 的 640x480 PIO 抓帧是 T07 之前的 LEGACY 通路");
        eprintln!("   当前 RTL 的 CTRL[4] 已保留（读回 0），不会再有 fb[] 帧。");
        eprintln!("   当前收帧请用 frame_grab（R10F → PGM）；");
        eprintln!("   只有确认在老位流上调试才加 --legacy。");
        return EX_DEV;
    }

    // ---- 1. 打开并映射 BAR0 前 4KB（offset=0、length ≤ 4KB）----
    // 刻意不经驱动的 ioctl / _user mmap：这里直接 mmap sysfs 的 resource0，
    // 换来每笔寄存器访问没有进程切换开销。代价是驱动侧的门禁在这条路径上无效，
    // 所以三条门禁（MAGIC / ≤4 dw / 全 F 熔断）由本工具自持（见文件头「五」）。
    // 先认设备（纯 sysfs 只读），确认是 PH1P100 EP 才去 open/mmap 它的 BAR0
    let rc = device_id_gate(&dev_path);
    if rc != EX_OK {
        return rc;
    }

    let file = match std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(sys::O_SYNC)
        .open(&dev_path)
    {
        Ok(f) => f,
        Err(e) => {
            eprintln!("!! 打不开 {}：{}", dev_path, sys::errno_str(&e));
            eprintln!("   （需要 root，且驱动已 insmod、设备已枚举；确认 EP 的 BDF）");
            return EX_DEV;
        }
    };
    let fd = file.as_raw_fd();
    if let Err(e) = sys::flock_exclusive_nb(fd) {
        eprintln!("!! {} 已被其它进程占用（flock 失败：{}），先停掉它", dev_path, sys::errno_str(&e));
        return EX_DEV;
    }
    let map = match Mmap::shared(fd, BAR_WIN_BYTES, PROT_READ | PROT_WRITE) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("!! mmap {} 失败：{}", dev_path, sys::errno_str(&e));
            return EX_DEV;
        }
    };

    let regs = Regs { ptr: map.as_ptr() as *mut u32, allf: Cell::new(0) };

    // ---- 2. MAGIC 门禁 ----
    let rc = regs.magic_gate("抓帧前");
    if rc != EX_OK {
        return rc;
    }

    // ---- 3. 读寄存器表快照 ----
    macro_rules! rd {
        ($off:expr) => {
            match regs.rd($off) {
                Ok(v) => v,
                Err(rc) => return rc,
            }
        };
    }

    let magic = rd!(R_MAGIC);
    let version = rd!(R_VERSION);
    let clk_mhz = rd!(R_CLK_MHZ);
    let width = rd!(R_WIDTH);
    let height = rd!(R_HEIGHT);
    let data_len = rd!(R_LEN);
    let ctrl = rd!(R_CTRL);
    let mut status = rd!(R_STATUS);
    let mut ptr = rd!(R_PTR);
    let f_cnt = rd!(R_FRAME);

    println!("=== 寄存器快照（0x40 MAGIC=0x{:08x} 自检通过）===", magic);
    println!(
        "  VERSION=0x{:08x}  CLK_MHZ={}  WIDTH={} HEIGHT={}  DATA_LEN={}",
        version, clk_mhz, width, height, data_len
    );
    println!(
        "  CTRL=0x{:08x}（bit0 enable={}, bit1 trigger={}, [3:2] pattern={}, bit4 cam_sel={}）",
        ctrl,
        if ctrl & CTRL_ENABLE != 0 { 1 } else { 0 },
        (ctrl >> 1) & 1,
        (ctrl & CTRL_PAT_MASK) >> CTRL_PAT_SHIFT,
        if ctrl & CTRL_CAM_SEL != 0 { 1 } else { 0 }
    );
    println!(
        "  STATUS=0x{:08x}（bit0 ready={}, bit1 busy={}）  FRAME_CNT={}  PTR={}",
        status,
        if status & STATUS_READY != 0 { 1 } else { 0 },
        (status >> 1) & 1,
        f_cnt,
        ptr
    );

    // ---- 4. 几何与帧缓存布局自检 ----
    if width == 0 || height == 0 || width.wrapping_mul(height) != FB_IMAGE_PIX {
        eprintln!(
            "!! 寄存器报的 {}x{} 与本工具的布局假设不符：本工具按 RTL 的\n   PTR_BYTES=0x{:X}（ep_bar_win.v:46）与 640x480 计算像素位置，\n   请先确认 FPGA 侧参数（img_frame_src 的 W/H：ph1p_pcie_ep_top.v:591-594）",
            width, height, FB_BYTES
        );
        return EX_DEV;
    }
    let w = width;
    let h = height;
    let pat_bits = ctrl & CTRL_PAT_MASK; // 写 CTRL 必须带上 [3:2]，见 ep_bar_win.v:157-161
    /*
     * 写 CTRL 会把整个字覆盖，而 [7:5] 是驱动探测时初始化好的位（bit5 = 每帧搬进 DDR3、
     * bit6 = 推帧给主机、bit7 = 推帧源）。本工具只改「写源」相关的位，上面那三位
     * 一律照原样带回去 —— 否则跑一次 cam_grab 就把链路的初始化拆了，又得人工补。
     */
    let keep_bits = ctrl & !(CTRL_ENABLE | CTRL_TRIGGER | CTRL_CAM_SEL | CTRL_PAT_MASK);

    // ---- 5. 选源（写 CTRL）----
    if src_cam {
        let ctrl_new = keep_bits | pat_bits | CTRL_ENABLE | CTRL_CAM_SEL;
        if let Err(rc) = regs.wr(R_CTRL, ctrl_new) {
            return rc;
        }
        let ctrl_rb = rd!(R_CTRL);
        println!("=== 写源：相机（CTRL ← 0x{:08x}，回读 0x{:08x}）===", ctrl_new, ctrl_rb);
        if ctrl_rb & CTRL_CAM_SEL == 0 {
            eprintln!(
                "!! CTRL 回读 bit4=0：cam_sel 没写进去（这条通道是借 ctrl_pattern[1] 过去的，ep_bar_win.v:105），不继续"
            );
            return EX_DEV;
        }
    } else {
        // enable=0 + trigger 单拍脉冲：出一帧就静止，避免边写边读撕裂
        let ctrl_new = keep_bits | pat_bits | CTRL_TRIGGER;
        if let Err(rc) = regs.wr(R_CTRL, ctrl_new) {
            return rc;
        }
        let ctrl_rb = rd!(R_CTRL);
        println!("=== 写源：测试图源（CTRL ← 0x{:08x}，回读 0x{:08x}）===", ctrl_new, ctrl_rb);
        if ctrl_rb & CTRL_CAM_SEL != 0 {
            eprintln!("!! CTRL 回读 bit4=1：cam_sel 没清掉，帧缓存写源还是相机");
            return EX_DEV;
        }
    }
    // trigger 是单拍脉冲（ep_bar_win.v:159-160 写入、:168-170 自动清），回读恒 0

    // ---- 6. 基准帧号（必须在选源之后读：0x4C 的含义由 CTRL[4] 决定，img_frame_src.v:104）----
    let f0 = rd!(R_FRAME);
    println!("  当前 FRAME_CNT={} 作为基准", f0);

    // ---- 7. 相机源：先看链路状态，没锁就直接退，不写图 ----
    if src_cam {
        let cam_a = rd!(R_CAM_A);
        let cam_b = rd!(R_CAM_B);
        let cam_af = rd!(R_CAM_A_FRAMES);
        let cam_bf = rd!(R_CAM_B_FRAMES);
        let cam_ap = rd!(R_CAM_A_PIX);
        let cam_bp = rd!(R_CAM_B_PIX);
        let cam_i2c = rd!(R_CAM_I2C);
        status = rd!(R_STATUS); // cam_sel 已选中，重读 STATUS

        cam_stat_decode(cam_a, "相机 A", &mut std::io::stdout());
        cam_stat_decode(cam_b, "相机 B", &mut std::io::stdout());
        println!("  帧缓存的写源是 A 路（ph1p_pcie_ep_top.v:899-917 只接了 w_a_*）");
        println!(
            "  A 路 VSYNC 计数={} 像素组计数={} | B 路 VSYNC={} 像素组={} | I2C 忙={} 上电计数={}",
            cam_af, cam_ap, cam_bf, cam_bp, cam_i2c & 1, cam_i2c >> 9
        );

        let rc = cam_stat_verdict(cam_a, "相机 A", &mut std::io::stderr());
        if rc != EX_OK {
            eprintln!("   ⇒ 不写图、直接退出（退出码 {}）。链路没锁时帧缓存里没有相机数据", rc);
            return rc;
        }
        if status & STATUS_READY == 0 {
            eprintln!("!! 相机 A 告警：STATUS.ready=0 → 到目前为止一帧都没写进帧缓存");
        }
    }

    // ---- 8. 等一帧（测试图源会重发 trigger）----
    let rc = wait_new_frame(
        &regs,
        f0,
        timeout_ms,
        !src_cam,
        pat_bits,
        if src_cam { "相机" } else { "测试图源" },
    );
    if rc != EX_OK {
        if src_cam {
            eprintln!("   ⇒ 不写图、直接退出（退出码 {}）：帧缓存里没有可用的相机帧", rc);
        }
        return rc;
    }

    // ---- 9. 确认写口空闲 + 记录 dump 前帧号 ----
    // busy=1 时帧缓存正被改写，这张图一定是半截的，所以先等它落下来；
    // 等不到就继续搬但标记 suspect（图照写、退出码 5 + 告警），不静默当成功。
    let mut suspect = false;
    {
        let t0 = now_ms();
        loop {
            status = rd!(R_STATUS);
            if status & STATUS_BUSY == 0 {
                break;
            }
            if now_ms() - t0 >= timeout_ms as u64 {
                eprintln!(
                    "!! STATUS.busy 连续 {} ms 为 1：帧缓存正在被写，这张图会是半截的（退出码 {}）",
                    timeout_ms, EX_INCONSISTENT
                );
                suspect = true;
                break;
            }
            msleep(POLL_MS);
        }
    }
    let f_pre = rd!(R_FRAME);

    // 搬运整帧之前再自检一次 MAGIC（读 0x40 不影响 PTR）
    let rc = regs.magic_gate("抓帧前（已就绪）");
    if rc != EX_OK {
        return rc;
    }

    // ---- 10. PTR 归零 + 整帧搬运 ----
    if let Err(rc) = regs.wr(R_PTR, 0) {
        return rc;
    }
    ptr = rd!(R_PTR);
    if ptr != 0 {
        eprintln!("!! 写 PTR=0 后回读 = {}（不为 0）→ 指针没清干净，不继续", ptr);
        return EX_DEV;
    }

    let mut fb = vec![0u32; FB_WORDS];
    println!("=== 搬运整帧：{} 字 × 4B（逐 dw 读 0x60，PTR 自动 +4）===", FB_WORDS);
    let t_dump0 = now_ms();
    for (i, slot) in fb.iter_mut().enumerate() {
        match regs.data_rd(i) {
            Ok(v) => *slot = v,
            Err(rc) => {
                eprintln!("   ⇒ 已读 {}/{} 字，帧数据不完整，不写图（退出码 {}）", i, FB_WORDS, rc);
                return rc;
            }
        }
    }
    let dump_ms = now_ms() - t_dump0;

    // ---- 11. 收尾自检 ----
    let rc = regs.magic_gate("搬运后");
    if rc != EX_OK {
        return rc;
    }
    let f_post = rd!(R_FRAME);
    let st_post = rd!(R_STATUS);
    let len_post = rd!(R_LEN);
    ptr = rd!(R_PTR);

    // ---- 12. 拼 PGM：像素 k ← 字 (k/4) 的第 (k%4) 字节（帧号不占缓存）----
    let npix_img = w as usize * h as usize;
    let npix_store = std::cmp::min(FB_PIXELS, npix_img);
    let mut img = vec![0u8; npix_img];
    fb_to_image(&fb, &mut img, npix_store);

    let mut fp = match std::fs::File::create(&out_path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("!! 打不开输出文件 {}：{}", out_path, sys::errno_str(&e));
            return EX_OUTPUT;
        }
    };
    let header = format!("P5\n{} {}\n255\n", w, h);
    let written = fp
        .write_all(header.as_bytes())
        .and_then(|_| fp.write_all(&img))
        .and_then(|_| fp.flush());
    if let Err(e) = written {
        eprintln!("!! 写 {} 失败：{}", out_path, sys::errno_str(&e));
        return EX_OUTPUT;
    }

    // ---- 13. 一致性校验 ----
    // 帧号只在 0x4C 里（不再占用帧缓存字 0），所以只靠「搬运期间帧号没变」
    // + （测试图源时）逐像素公式自检来判一致性。
    let torn = f_post != f_pre;
    // 测试图源的像素公式用的是「帧末才 +1」的那个帧号 ⇒ 生成这一帧时用的是 f_pre - 1
    let pattern_frame = f_pre.wrapping_sub(1);

    println!("\n=== 结果 ===");
    println!(
        "  输出文件        : {}（P5 二进制 PGM，{}x{} 灰度，{} 字节像素）",
        out_path, w, h, npix_img
    );
    println!(
        "  写源            : {}（CTRL[4]={}）",
        if src_cam { "相机" } else { "测试图源" },
        if src_cam { 1 } else { 0 }
    );
    println!("  帧号（0x4C）    : {}（帧缓存字 0..{} 全是像素，帧号不再占缓存）",
             f_pre, FB_WORDS - 1);
    println!(
        "  FRAME_CNT       : 抓帧前 {} → 搬运前 {} → 搬运后 {}  {}",
        f0,
        f_pre,
        f_post,
        if torn { "← 搬运期间变过！" } else { "（搬运期间未变）" }
    );
    println!(
        "  DATA_LEN        : 抓帧前 {} / 搬运后 {}（期望 {} = {}x{}）",
        data_len,
        len_post,
        w * h,
        w,
        h
    );
    if src_cam {
        println!(
            "                    （相机源的 DATA_LEN 是常量 {}，cam_fb_writer.v:116：\n                     它不能用来判「有没有出帧」，判帧只看 FRAME_CNT）",
            w * h
        );
    }
    println!(
        "  STATUS（搬运后）: 0x{:08x}（bit0 ready={}, bit1 busy={}）  PTR（搬运后）={}",
        st_post,
        if st_post & STATUS_READY != 0 { 1 } else { 0 },
        (st_post >> 1) & 1,
        ptr
    );
    println!(
        "  读到全 F 次数   : {}（大于 0 会在读到时立即熔断退出，走不到这里）",
        regs.allf.get()
    );
    println!(
        "  搬运耗时        : {} ms（{:.1} MB/s，PIO 单 dw 读）",
        dump_ms,
        if dump_ms != 0 {
            FB_BYTES as f64 / dump_ms as f64 / 1000.0
        } else {
            0.0
        }
    );
    println!(
        "  布局说明        : 字 0..{} 全是像素（帧缓存 {} 字节 = 一帧 {}x{}），",
        FB_WORDS - 1,
        FB_BYTES,
        w,
        h
    );
    println!(
        "                    字 k 装 x=k*4..k*4+3（低字节 = 最左像素）；帧号只看 0x4C"
    );

    let mut bad = 0usize;
    if !src_cam {
        bad = pattern_check(
            &img,
            npix_store,
            w as usize,
            pattern_frame,
            "若整帧不符，先查 RTL 布局/搬运；零散几个不符，多半是撕裂",
        );
    }

    println!(
        "  一致性          : {}",
        if torn || bad != 0 || suspect {
            "**不通过**（图已写出，但请勿当可信帧用；原因见上面的告警）"
        } else {
            "OK（搬运期间帧号未变；测试图源逐像素公式自检通过）"
        }
    );

    if src_cam && torn {
        eprintln!(
            "!! 搬运期间 FRAME_CNT 变了（{} → {}）：相机在连续出帧，这张图很可能是两帧的混合（PIO 带宽 ~2.6MB/s，一帧需 {} ms）。图已写出，请以帧号为准",
            f_pre, f_post, dump_ms
        );
    }

    if torn || bad != 0 || suspect {
        EX_INCONSISTENT
    } else {
        EX_OK
    }
}

fn main() {
    let code = run();
    anlogic_pci_tools::exit_with(code);
}
