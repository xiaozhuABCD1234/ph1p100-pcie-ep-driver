/* ==========================================================================
 * cam_grab.c — 从 PH1P100 PCIe EP 的 BAR0 抓一帧 640x480 灰度图，写成 PGM
 *
 * 本文件是对原手写草稿的重写：抓帧时序、寄存器语义、状态字判据全部按工程内 RTL
 * 实现（逐条标 file:line），访问方式遵守本文件「五、访问规约」一节所列各条。
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
 *   字 0        = 帧号（img_frame_src.v:14-15/:166-171；相机同布局 cam_fb_writer.v:44-49）
 *   字 1..76799 = 像素，4 像素/32bit 字、小端（x 小的在低字节，
 *                 img_frame_src.v:12-15、cam_fb_writer.v:125）
 *   ⇒ 像素 k 在字 (k/4 + 1) 的第 (k%4) 字节；一帧 640x480 = 307200 像素，
 *     但帧缓存只有 76800 字（307200 字节，ep_bar_win.v:46 PTR_BYTES=0x4B000），
 *     字 0 被帧号占掉 4 字节 ⇒ 实际只存得下 76799 字 = 307196 个像素，
 *     最后 4 个像素（x=636..639, y=479）两个写源都不写
 *     （cam_fb_writer.v:47-49 原文「末 4 像素与测试图源一样不写」；
 *      测试图发生器写完 widx==WORDS-1 即结束：img_frame_src.v:181-186）。
 *   两个写源的帧号约定差 1，工具会分别核对：
 *     - 测试图：字 0 = 本帧框号，帧结束时 FRAME_CNT 才 +1
 *               （:170 写 frame_cnt_t，:185 才自增）⇒ 读回 FRAME_CNT == 字0 + 1
 *     - 相机  ：SOF 时写 frame_cnt+1，EOF 时 frame_cnt++（cam_fb_writer.v:271/:279）
 *               ⇒ 读回 FRAME_CNT == 字0
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
 * 用法：./cam_grab [-s cam|pattern] [-d /sys/…/resource0] [-t 超时ms] [输出.pgm]
 *       ./cam_grab --decode 0xcdee0020      # 只把状态字解码成人话，不碰硬件
 * ========================================================================== */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

/* ---------------------------------------------------------------- 常量 */

#define DEF_DEV    "/sys/bus/pci/devices/0000:01:00.0/resource0"
#define DEF_OUT    "frame.pgm"
#define DEF_TIMEOUT_MS 3000u

#define BAR_WIN_BYTES  4096u        /* mmap 只取 4KB（帧数据走 0x60 口搬运） */
#define MAGIC_EXPECT   0x50434946u  /* ep_bar_win.v:216 + 参数 :43 */
#define VAL_ALLF       0xffffffffu
#define VAL_DEADBEEF   0xdeadbeefu  /* ep_bar_win.v:240 default 分支 */

#define EP_VENDOR_ID   0x1edbu   /* fpga/al_ip/pcie_ep_core/.params_pcie_ep_core.txt:55 */
#define EP_DEVICE_ID   0xabcdu   /* 同上 :56（厂商驱动匹配表 anlogic_pci_drv.c:50-57 一致） */

#define FB_BYTES       0x4B000u                  /* ep_bar_win.v:46 PTR_BYTES（640*480） */
#define FB_WORDS       (FB_BYTES / 4u)           /* 76800 字 */
#define FB_PIXELS      (FB_WORDS * 4u - 4u)      /* 字 1..76799 → 307196 个像素 */
#define FB_IMAGE_PIX   (640u * 480u)             /* 期望的 W*H */

/* 寄存器表：只有 ep_bar_win.v 读 mux（:216-239）登记过的 offset 才允许访问 */
#define R_MAGIC        0x40u   /* :216 */
#define R_CTRL         0x44u   /* :217 / 写语义 :157-161 / trigger 自动清 :168-170 */
#define R_STATUS       0x48u   /* :218 {…, busy, ready} */
#define R_FRAME        0x4Cu   /* :219 */
#define R_WIDTH        0x50u   /* :220 */
#define R_HEIGHT       0x54u   /* :221 */
#define R_LEN          0x58u   /* :222 */
#define R_PTR          0x5Cu   /* :223 / 写 :162 */
#define R_DATA         0x60u   /* :224，读后 PTR += 4（:111/:152/:175-181） */
#define R_VERSION      0x64u   /* :225 */
#define R_CLK_MHZ      0x68u   /* :226 */
#define R_CAM_A        0x80u   /* :232 cam_stat[31:0]      = A 路状态 */
#define R_CAM_B        0x84u   /* :233 cam_stat[63:32]     = B 路状态 */
#define R_CAM_A_FRAMES 0x88u   /* :234 cam_stat[95:64]     = A 路 VSYNC 计数 */
#define R_CAM_B_FRAMES 0x8Cu   /* :235 cam_stat[127:96]    = B 路 VSYNC 计数 */
#define R_CAM_A_PIX    0x90u   /* :236 cam_stat[159:128]   = A 路像素组计数 */
#define R_CAM_B_PIX    0x94u   /* :237 cam_stat[191:160]   = B 路像素组计数 */
#define R_CAM_I2C      0x98u   /* :238 cam_stat[223:192]   = {r_pwr_cnt2,8'd0,i2c_busy} */
#define R_TAP          0x9Cu   /* :239 cam_tap_cfg {manual_en,tap[5:0]} */

#define CTRL_ENABLE    (1u << 0)
#define CTRL_TRIGGER   (1u << 1)
#define CTRL_PAT_SHIFT 2u
#define CTRL_PAT_MASK  (3u << CTRL_PAT_SHIFT)
#define CTRL_CAM_SEL   (1u << 4)

#define STATUS_READY   (1u << 0)
#define STATUS_BUSY    (1u << 1)

#define POLL_MS        10u     /* 轮询间隔 ≥ 1ms */
#define RETRIGGER_MS   100u    /* 测试图源重发 trigger 的最小间隔（IDLE_CYCLE≈16ms@62MHz）*/

enum exit_code {
    EX_OK            = 0,
    EX_USAGE         = 1,
    EX_DEV           = 2,
    EX_CAM_NOT_READY = 3,
    EX_NO_FRAME      = 4,
    EX_INCONSISTENT  = 5,
    EX_OUTPUT        = 6,
};

/* ---------------------------------------------------------------- 全局 */

static volatile uint32_t *g_bar;   /* 只映射 BAR0 前 4KB */
static unsigned           g_allf;  /* 读到 0xffffffff 的次数（命中即熔断，正常恒 0）*/

/* ---------------------------------------------------------------- 小工具 */

static void msleep(unsigned ms)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR)
        ;
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

static int off_in_list(const uint32_t *list, size_t n, uint32_t off)
{
    for (size_t i = 0; i < n; i++)
        if (list[i] == off)
            return 1;
    return 0;
}

/* 允许「读」的 offset（0x60 除外：读它就会动 PTR，只能走 data_rd） */
static const uint32_t k_rd_ok[] = {
    R_MAGIC, R_CTRL, R_STATUS, R_FRAME, R_WIDTH, R_HEIGHT, R_LEN, R_PTR,
    R_VERSION, R_CLK_MHZ, 0x6Cu, 0x70u, 0x74u, 0x78u, 0x7Cu,
    R_CAM_A, R_CAM_B, R_CAM_A_FRAMES, R_CAM_B_FRAMES,
    R_CAM_A_PIX, R_CAM_B_PIX, R_CAM_I2C, R_TAP,
};
/* 允许「写」的 offset：只用得到 CTRL 与 PTR（ep_bar_win.v:157-165） */
static const uint32_t k_wr_ok[] = { R_CTRL, R_PTR };

static int off_rd_ok(uint32_t off) { return off_in_list(k_rd_ok, sizeof k_rd_ok / sizeof k_rd_ok[0], off); }
static int off_wr_ok(uint32_t off) { return off_in_list(k_wr_ok, sizeof k_wr_ok / sizeof k_wr_ok[0], off); }

/* ---------------------------------------------------------------- MMIO */

/* 读一个登记过的寄存器：32 位对齐、单 dw。
 * 0xffffffff → 熔断；0xDEADBEEF → 说明打到了未译码地址（:240）。 */
static int reg_rd(uint32_t off, uint32_t *out)
{
    uint32_t v;

    if (!off_rd_ok(off)) {
        fprintf(stderr, "!! 程序内部错误：试图读未登记的 offset 0x%02x\n", (unsigned)off);
        return EX_DEV;
    }
    v = g_bar[off >> 2];
    if (v == VAL_ALLF) {
        g_allf++;
        fprintf(stderr, "!! 读 0x%02x 得到 0xffffffff：设备已不可用，立即停止，"
                        "不再发任何 MMIO\n",
                (unsigned)off);
        return EX_DEV;
    }
    if (v == VAL_DEADBEEF) {
        fprintf(stderr, "!! 读 0x%02x 得到 0xdeadbeef：FPGA 回的是未译码默认值"
                        "（ep_bar_win.v:240），该 offset 不该被访问\n", (unsigned)off);
        return EX_DEV;
    }
    *out = v;
    return EX_OK;
}

static int reg_wr(uint32_t off, uint32_t v)
{
    if (!off_wr_ok(off)) {
        fprintf(stderr, "!! 程序内部错误：试图写未登记的 offset 0x%02x\n", (unsigned)off);
        return EX_DEV;
    }
    g_bar[off >> 2] = v;
    __sync_synchronize();
    return EX_OK;
}

/* 读 0x60 数据口：返回当前 PTR 处一个 dw，PTR 自动 +4（ep_bar_win.v:111/:152/:175-181）。
 * 只用单 dw：① 单次访问不得超过 4 个 dw；② 预取 4 拍轮转是给「访问间隔远大于 4 拍」设计的
 * （ep_bar_win.v:186-203），一次 MRd 往返远大于此；③ 同地址连续读不保证同值。 */
static int data_rd(size_t idx, uint32_t *out)
{
    uint32_t v = g_bar[R_DATA >> 2];

    if (v == VAL_ALLF) {
        g_allf++;
        fprintf(stderr, "!! 读 0x60 数据口第 %zu 个字得到 0xffffffff：设备已不可用，立即停止\n", idx);
        return EX_DEV;
    }
    *out = v;
    return EX_OK;
}

/* 任何一轮访问之前先自检 MAGIC。
 * 读 0x40 不会动 PTR（只有 0x60 会），所以抓帧前后各查一次是安全的。 */
static int magic_gate(const char *when)
{
    uint32_t m;
    int rc = reg_rd(R_MAGIC, &m);

    if (rc != EX_OK)
        return rc;
    if (m != MAGIC_EXPECT) {
        fprintf(stderr, "!! MAGIC=0x%08x ≠ 0x%08x（%s）：设备未就绪或链路已断，"
                        "不继续碰任何寄存器、不写图\n",
                (unsigned)m, (unsigned)MAGIC_EXPECT, when);
        return EX_DEV;
    }
    return EX_OK;
}

/* ------------------------------------------------- 相机状态字解码 / 判据 */

/* 逐位打印（位定义 ph1p_pcie_ep_top.v:1036-1038），out 可以是 stdout 或 stderr */
static void cam_stat_decode(uint32_t s, const char *tag, FILE *out)
{
    unsigned bclk  = (s >> 0) & 1u;
    unsigned lane  = (s >> 1) & 0xfu;
    unsigned cfg   = (s >> 5) & 1u;
    unsigned buse  = (s >> 6) & 1u;
    unsigned adone = (s >> 7) & 1u;
    unsigned infr  = (s >> 8) & 1u;
    unsigned inln  = (s >> 9) & 1u;
    unsigned pv    = (s >> 10) & 1u;
    unsigned rsvd  = (s >> 11) & 0x1fu;
    unsigned tap   = (s >> 16) & 0x3fu;
    unsigned slow  = (s >> 22) & 1u;
    unsigned rstn  = (s >> 23) & 1u;
    unsigned scl   = (s >> 24) & 1u;
    unsigned sda   = (s >> 25) & 1u;
    unsigned tag6  = (s >> 26) & 0x3fu;

    fprintf(out, "  %s 状态字 0x%08x 逐位解码（位定义 ph1p_pcie_ep_top.v:1036-1038）:\n",
            tag, (unsigned)s);
    fprintf(out, "    bit0       字节时钟在跳            = %u%s\n", bclk,
            bclk ? "" : "   ← 软 PHY 字节时钟没有活动，MIPI 高速时钟没进来");
    fprintf(out, "    bit[4:1]   lane_valid[3:0]         = 0x%x%s\n", lane,
            lane == 0xfu ? "（4 条 lane 全对齐）"
                         : (lane == 0u ? "   ← 没有任何一条 lane 字节对齐，MIPI 链路未锁定"
                                       : "   ← 只有部分 lane 对齐"));
    fprintf(out, "    bit5       I2C 配置完成 cfg_done   = %u%s\n", cfg,
            cfg ? "" : "   ← IMX415 寄存器还没配置完");
    fprintf(out, "    bit6       I2C 总线错误 bus_err    = %u\n", buse);
    fprintf(out, "    bit7       auto_delay_done         = %u\n", adone);
    fprintf(out, "    bit8       in_frame（帧内电平）    = %u\n", infr);
    fprintf(out, "    bit9       in_line（行首脉冲）     = %u\n", inln);
    fprintf(out, "    bit10      收到过像素组 pv_seen    = %u\n", pv);
    fprintf(out, "    bit[15:11] 保留                    = 0x%x\n", rsvd);
    fprintf(out, "    bit[21:16] 当前 IDELAY tap         = %u\n", tap);
    fprintf(out, "    bit22      xclr/r_rst_n_slow 已放开= %u\n", slow);
    fprintf(out, "    bit23      csi_rst_n               = %u\n", rstn);
    fprintf(out, "    bit24/25   SCL / SDA 电平          = %u / %u\n", scl, sda);
    fprintf(out, "    bit[31:26] 固定标签                = 0x%02x（期望 0x33）\n", tag6);
}

/* 判据：返回 EX_OK = 可以继续抓帧；否则返回 EX_CAM_NOT_READY（已打印明确原因）。
 * 必死项按任务要求：字节时钟没跳 / lane_valid==0 / cfg_done==0；
 * bus_err、部分 lane、没收到过像素组只告警（帧号与一致性校验还会兜底）。 */
static int cam_stat_verdict(uint32_t s, const char *tag, FILE *out)
{
    unsigned bclk = (s >> 0) & 1u;
    unsigned lane = (s >> 1) & 0xfu;
    unsigned cfg  = (s >> 5) & 1u;
    unsigned buse = (s >> 6) & 1u;
    unsigned pv   = (s >> 10) & 1u;
    int fatal = 0;

    if (!bclk) {
        fprintf(out, "!! %s 链路未就绪：字节时钟没有在跳（bit0=0）→ 软 PHY 收不到 "
                     "MIPI 高速时钟，后面无论等多久都不会有帧\n", tag);
        fatal = 1;
    }
    if (lane == 0u) {
        fprintf(out, "!! %s 链路未就绪：lane_valid=0（bit[4:1]=0）→ 4 条 lane 全部"
                     "字节未对齐，MIPI 链路没有锁（PHY/接线/摄像头供电/参考时钟）\n", tag);
        fatal = 1;
    }
    if (!cfg) {
        fprintf(out, "!! %s 链路未就绪：cfg_done=0（bit5=0）→ IMX415 的 I2C 寄存器"
                     "还没配置完，帧不可能出来\n", tag);
        fatal = 1;
    }
    if (buse)
        fprintf(out, "!! %s 告警：I2C bus_err=1（bit6）→ 传感器配置过程出过总线错误\n", tag);
    if (lane != 0u && lane != 0xfu)
        fprintf(out, "!! %s 告警：只有部分 lane 对齐（lane_valid=0x%x，期望 0xf）\n", tag, lane);
    if (!pv)
        fprintf(out, "!! %s 告警：还没收到过任何像素组（bit10=0）\n", tag);

    return fatal ? EX_CAM_NOT_READY : EX_OK;
}

/* ------------------------------------------------------- 设备身份门禁 */

static int read_sysfs_hex(const char *dir, const char *name, unsigned *out)
{
    char  path[PATH_MAX];
    FILE *f;

    if (snprintf(path, sizeof path, "%s/%s", dir, name) >= (int)sizeof path)
        return -1;
    f = fopen(path, "r");
    if (!f)
        return -1;
    if (fscanf(f, "%x", out) != 1) {
        fclose(f);
        return -1;
    }
    fclose(f);
    return 0;
}

/* 发任何 MMIO 之前先确认「这确实是 PH1P100 EP」：0x40/0x44 这种读在别的设备上
 * 是「读它的私有寄存器」，必须避免。只读 sysfs，不碰设备本身。
 * 读不到 vendor/device（不是 sysfs 路径）时只告警放行，由使用者负责。 */
static int device_id_gate(const char *dev_path)
{
    char     dir[PATH_MAX];
    unsigned vid = 0, did = 0;
    size_t   dlen;
    const char *slash = strrchr(dev_path, '/');

    if (!slash || slash == dev_path || strlen(dev_path) >= sizeof dir) {
        fprintf(stderr, "!! 警告：%s 不像 sysfs 里的 resourceN，跳过设备身份自检\n"
                        "   （请自行确认它确实是 PH1P100 EP 的 BAR0 映射）\n", dev_path);
        return EX_OK;
    }
    dlen = (size_t)(slash - dev_path);
    memcpy(dir, dev_path, dlen);
    dir[dlen] = '\0';

    if (read_sysfs_hex(dir, "vendor", &vid) != 0 ||
        read_sysfs_hex(dir, "device", &did) != 0) {
        fprintf(stderr, "!! 警告：读不到 %s/vendor|device，跳过设备身份自检\n", dir);
        return EX_OK;
    }
    if (vid != EP_VENDOR_ID || did != EP_DEVICE_ID) {
        fprintf(stderr, "!! %s 的 PCI ID 是 %04x:%04x，不是 PH1P100 EP 的 %04x:%04x\n"
                        "   → 拒绝在它上面发任何 MMIO（默认路径是给板子的，别的机器上\n"
                        "   0000:01:00.0 往往是另一块设备）。出处：fpga/al_ip/pcie_ep_core/\n"
                        ".params_pcie_ep_core.txt:55-56（PCIE_CFG_PF0_*）\n",
                dev_path, vid, did, (unsigned)EP_VENDOR_ID, (unsigned)EP_DEVICE_ID);
        return EX_DEV;
    }
    printf("  设备身份：%04x:%04x（PH1P100 EP ✓）\n", vid, did);
    return EX_OK;
}

/* ------------------------------------------------------------- 帧就绪等待 */

static int read_status(uint32_t *st)   { return reg_rd(R_STATUS, st); }
static int read_frame(uint32_t *f)     { return reg_rd(R_FRAME, f); }

/* 等 0x4C 帧号相对 f0 变化（并且 busy 落回 0）。
 * retrigger=1 时每 RETRIGGER_MS 重发一次 trigger：
 *   img_frame_src.v:133 的 start 被 (ready_t==0 || idle>=IDLE_CYCLE) 卡着，
 *   IDLE_CYCLE=1000000（ph1p_pcie_ep_top.v:594）≈16ms@62MHz，上一帧刚出完时
 *   第一发 trigger 可能被吞掉；帧中重发是安全的：start 只在 !busy_t 时求值
 *   （img_frame_src.v:150-152），不会插帧。 */
static int wait_new_frame(uint32_t f0, unsigned timeout_ms, int retrigger,
                          uint32_t ctrl_base, const char *tag)
{
    uint64_t t0 = now_ms(), last_pulse = t0;

    for (;;) {
        uint32_t f, st;
        int rc = read_frame(&f);

        if (rc != EX_OK)
            return rc;
        if (f != f0) {
            rc = read_status(&st);
            if (rc != EX_OK)
                return rc;
            if (!(st & STATUS_BUSY)) {
                printf("  %s：帧号 %u → %u，busy=0，一帧写完（%llu ms）\n", tag,
                       (unsigned)f0, (unsigned)f, (unsigned long long)(now_ms() - t0));
                return EX_OK;
            }
        }
        if (now_ms() - t0 >= timeout_ms) {
            uint32_t fl, stl;
            fprintf(stderr, "!! %s：等了 %u ms 帧号仍是 %u（起等值 %u），没有新帧\n",
                    tag, timeout_ms, (unsigned)f, (unsigned)f0);
            if (read_status(&stl) == EX_OK)
                fprintf(stderr, "   最后 STATUS=0x%08x（bit0 ready=%u, bit1 busy=%u）\n",
                        (unsigned)stl, (unsigned)(stl & STATUS_READY),
                        (unsigned)((stl >> 1) & 1u));
            if (read_frame(&fl) == EX_OK && fl != f)
                fprintf(stderr, "   再读 0x4C = %u（期间又变了，说明帧在出但节奏很慢）\n",
                        (unsigned)fl);
            return EX_NO_FRAME;
        }
        if (retrigger && now_ms() - last_pulse >= RETRIGGER_MS) {
            if (reg_wr(R_CTRL, ctrl_base | CTRL_TRIGGER) != EX_OK)
                return EX_DEV;
            last_pulse = now_ms();
        }
        msleep(POLL_MS);
    }
}

/* ------------------------------------------------- 帧缓存 → PGM 像素映射 */

/* 像素 k ← 帧缓存字 (k/4 + 1) 的第 (k%4) 字节，低字节 = 最左像素：
 *   - 字 0 是帧号，像素字从字 1 开始（img_frame_src.v:14-15/:166-171）
 *   - 4 像素/32bit 字、小端（img_frame_src.v:12-15；相机同布局 cam_fb_writer.v:125）
 *   - 帧缓存只有 FB_PIXELS 个像素位置，末尾 (npix_img - npix_store) 个像素留 0
 * 剩下的字节保持 0，由调用者先 memset。 */
static void fb_to_image(const uint32_t *fb, unsigned char *img, size_t npix_img,
                        size_t npix_store)
{
    for (size_t k = 0; k < npix_store && k < npix_img; k++) {
        uint32_t w = fb[k / 4u + 1u];
        img[k] = (unsigned char)((w >> (8u * (unsigned)(k % 4u))) & 0xffu);
    }
}

/* --------------------------------------------------------- 测试图源自检 */

/* 测试图源的像素公式（img_frame_src.v:15-17）：
 *   pix = x[7:0] + y[7:0] + frame_cnt[7:0] + PIX_ADD，顶层没覆盖 PIX_ADD（默认 0）
 *   （ph1p_pcie_ep_top.v:591-595 只传了 W/H/IDLE_CYCLE）
 * 逐像素核对，既验证搬运正确性，也验证「字 0 = 帧号、像素从字 1 开始」这个布局理解。 */
static size_t pattern_check(const unsigned char *img, size_t n, unsigned W,
                            uint32_t frame, const char *what)
{
    size_t bad = 0;

    for (size_t k = 0; k < n; k++) {
        unsigned x = (unsigned)(k % W);
        unsigned y = (unsigned)(k / W);
        unsigned char want = (unsigned char)((x + y + (frame & 0xffu)) & 0xffu);
        if (img[k] != want) {
            if (bad < 4)
                fprintf(stderr, "!! 测试图自检不符：像素 %zu（x=%u y=%u）期望 0x%02x 实际 0x%02x\n",
                        k, x, y, (unsigned)want, (unsigned)img[k]);
            bad++;
        }
    }
    printf("  模式自检        : %zu/%zu 像素与 (x+y+帧号)&0xff 一致", n - bad, n);
    if (bad)
        printf("  ← %zu 个不符（%s）", bad, what);
    printf("\n");
    return bad;
}

/* ---- --selftest：主机侧「字0=帧号 / 像素从字1起」映射与自检逻辑的离线自证 ----
 * 合成一帧（按 RTL 公式生成帧缓存内容）喂给本工具真正用的那两段代码：
 *   ① 本工具的映射必须 0 处不符；② 旧草稿那种「把字 0 也当像素」的错位映射
 *   必须被查出不符。不访问任何硬件。*/
static int self_test(void)
{
    const unsigned W = 640u, H = 480u;
    const size_t   npix_img   = (size_t)W * H;
    const size_t   npix_store = FB_PIXELS;
    const uint32_t frame      = 7u;
    uint32_t      *fb  = calloc(FB_WORDS, sizeof *fb);
    unsigned char *img = malloc(npix_img);
    int            ret = EX_OK;

    if (!fb || !img) {
        fprintf(stderr, "!! selftest 内存分配失败\n");
        free(fb);
        free(img);
        return EX_OUTPUT;
    }

    printf("=== --selftest：主机侧像素映射/自检逻辑离线自证（未访问硬件）===\n");
    printf("  合成帧：字 0 = 帧号 %u；像素字 1..%u = (x+y+帧号)&0xff\n",
           (unsigned)frame, (unsigned)(FB_WORDS - 1u));
    fb[0] = frame;
    for (size_t k = 0; k < npix_store; k++) {
        unsigned        px = (unsigned)(k % W), py = (unsigned)(k / W);
        unsigned char   p  = (unsigned char)((px + py + (frame & 0xffu)) & 0xffu);
        fb[k / 4u + 1u] |= (uint32_t)p << (8u * (unsigned)(k % 4u));
    }

    memset(img, 0, npix_img);
    fb_to_image(fb, img, npix_img, npix_store);
    if (pattern_check(img, npix_store, W, frame, "合成数据本不该不符")) {
        fprintf(stderr, "!! 自证失败：本工具的映射/自检逻辑对合成帧报不符\n");
        ret = EX_INCONSISTENT;
    }

    /* 反证：字 0 也当像素（小端机上的原始字节铺开）→ 必须被抓出来 */
    memset(img, 0, npix_img);
    memcpy(img, fb, npix_img);
    if (pattern_check(img, npix_store, W, frame, "这是旧草稿的映射，本就该不符") == 0) {
        fprintf(stderr, "!! 自证失败：自检没能查出已知的 4 像素错位\n");
        ret = EX_INCONSISTENT;
    }

    free(fb);
    free(img);
    printf("  结论：映射与自检逻辑自证%s\n", ret == EX_OK ? "通过" : "**失败**");
    return ret;
}

/* ------------------------------------------------------------- 使用说明 */

static void usage(const char *prog)
{
    printf(
        "用法: %s [选项] [输出.pgm]\n"
        "\n"
        "从 PH1P100 PCIe EP 的 BAR0 寄存器表抓一帧 640x480 灰度图并存成 PGM。\n"
        "访问规矩：先读 0x40 MAGIC 自检、只碰登记过的\n"
        "offset、单次单 dw、任一读拿到 0xffffffff 立即熔断退出。\n"
        "\n"
        "选项:\n"
        "  -s, --source cam|pattern   写源（默认 cam；对应 CTRL[4]=1/0）\n"
        "  -d, --dev PATH             BAR0 资源节点（默认 %s）\n"
        "  -t, --timeout MS           等新帧超时（默认 %u，最小 50）\n"
        "      --decode 0xXXXXXXXX    只把相机状态字逐位解码成人话，不碰任何硬件\n"
        "                             （可给多个；0x80=A 路、0x84=B 路）\n"
        "      --selftest             离线自证主机侧像素映射/自检逻辑（合成帧，不碰硬件）\n"
        "  -h, --help                 本说明\n"
        "\n"
        "测试图源是「单帧触发」：CTRL=[3:2]|bit1（enable=0，出一帧就静止），\n"
        "所以帧缓存不会被边写边读地撕裂。相机源由链路连续出帧，搬运期间若帧号\n"
        "变化（或写口一直忙）会判为可疑：图仍写出，但退出码 5 + 告警，不当成功。\n"
        "\n"
        "退出码: 0 成功 / 1 用法 / 2 设备不可用 / 3 相机链路未就绪（不写图）/\n"
        "        4 等不到新帧（不写图）/ 5 图已写出但校验不过 / 6 输出文件错误\n",
        prog, DEF_DEV, DEF_TIMEOUT_MS);
}

static int parse_source(const char *s, int *out)
{
    if (!strcmp(s, "cam") || !strcmp(s, "camera") || !strcmp(s, "1"))
        *out = 1;
    else if (!strcmp(s, "pattern") || !strcmp(s, "test") || !strcmp(s, "0"))
        *out = 0;
    else
        return -1;
    return 0;
}

/* ----------------------------------------------------------------- main */

int main(int argc, char **argv)
{
    const char *dev_path = DEF_DEV;
    const char *out_path = NULL;
    int         src_cam  = 1;
    unsigned    timeout_ms = DEF_TIMEOUT_MS;
    uint32_t    decode_vals[16];
    int         n_decode = 0;
    int         want_selftest = 0;

    static const struct option longopts[] = {
        { "source",  required_argument, NULL, 's' },
        { "dev",     required_argument, NULL, 'd' },
        { "timeout", required_argument, NULL, 't' },
        { "decode",  required_argument, NULL, 1   },
        { "selftest", no_argument,      NULL, 2   },
        { "help",    no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };

    for (;;) {
        int c = getopt_long(argc, argv, "s:d:t:h", longopts, NULL);
        if (c == -1)
            break;
        switch (c) {
        case 's':
            if (parse_source(optarg, &src_cam) != 0) {
                fprintf(stderr, "!! --source 只接受 cam 或 pattern（收到 '%s'）\n", optarg);
                return EX_USAGE;
            }
            break;
        case 'd':
            dev_path = optarg;
            break;
        case 't': {
            char *end = NULL;
            unsigned long v = strtoul(optarg, &end, 0);
            if (!end || *end || v == 0 || v > 600000UL) {
                fprintf(stderr, "!! --timeout 需要 1..600000 的毫秒数（收到 '%s'）\n", optarg);
                return EX_USAGE;
            }
            timeout_ms = (unsigned)v;
            break;
        }
        case 1: {
            char *end = NULL;
            unsigned long v;
            if (n_decode >= (int)(sizeof decode_vals / sizeof decode_vals[0])) {
                fprintf(stderr, "!! --decode 最多 16 个值\n");
                return EX_USAGE;
            }
            v = strtoul(optarg, &end, 0);
            if (!end || *end || v > 0xffffffffUL) {
                fprintf(stderr, "!! --decode 需要 32bit 十六进制值（收到 '%s'）\n", optarg);
                return EX_USAGE;
            }
            decode_vals[n_decode++] = (uint32_t)v;
            break;
        }
        case 'h':
            usage(argv[0]);
            return EX_OK;
        case 2:
            want_selftest = 1;
            break;
        default:
            usage(argv[0]);
            return EX_USAGE;
        }
    }
    /* 位置参数：输出文件名；兼容旧写法 `cam_grab out.pgm pattern` */
    for (int i = optind; i < argc; i++) {
        int s;
        if ((!strcmp(argv[i], "cam") || !strcmp(argv[i], "pattern")) &&
            parse_source(argv[i], &s) == 0)
            src_cam = s;
        else if (!out_path)
            out_path = argv[i];
        else {
            fprintf(stderr, "!! 多余的参数：'%s'\n", argv[i]);
            return EX_USAGE;
        }
    }
    if (!out_path)
        out_path = DEF_OUT;
    if (timeout_ms < 50u)
        timeout_ms = 50u;

    /* ---- --decode / --selftest：纯离线，不打开设备、不做任何 MMIO ---- */
    if (want_selftest)
        return self_test();
    if (n_decode) {
        printf("=== --decode 离线解码（未访问任何硬件）===\n");
        for (int i = 0; i < n_decode; i++) {
            printf("--- 状态字 #%d（若来自 0x80 则是 A 路，0x84 则是 B 路）\n", i + 1);
            cam_stat_decode(decode_vals[i], "相机", stdout);
            (void)cam_stat_verdict(decode_vals[i], "相机", stdout);
            printf("\n");
        }
        return EX_OK;
    }

    /* ---- 1. 打开并映射 BAR0 前 4KB（offset=0、length ≤ 4KB） ---- */
    int fd, rc;

    /* 先认设备（纯 sysfs 只读），确认是 PH1P100 EP 才去 open/mmap 它的 BAR0 */
    if ((rc = device_id_gate(dev_path)) != EX_OK)
        return rc;

    fd = open(dev_path, O_RDWR | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "!! 打不开 %s：%s\n", dev_path, strerror(errno));
        fprintf(stderr, "   （需要 root，且驱动已 insmod、设备已枚举；确认 EP 的 BDF）\n");
        return EX_DEV;
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {   /* 一次只允许一个进程 */
        fprintf(stderr, "!! %s 已被其它进程占用（flock 失败：%s），先停掉它\n",
                dev_path, strerror(errno));
        close(fd);
        return EX_DEV;
    }
    g_bar = mmap(NULL, BAR_WIN_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (g_bar == MAP_FAILED) {
        fprintf(stderr, "!! mmap %s 失败：%s\n", dev_path, strerror(errno));
        close(fd);
        return EX_DEV;
    }

    /* ---- 2. MAGIC 门禁---- */
    rc = magic_gate("抓帧前");
    if (rc != EX_OK)
        return rc;

    /* ---- 3. 读寄存器表快照 ---- */
    uint32_t magic, version, clk_mhz, width, height, data_len, ctrl, status, ptr, f_cnt;
    if ((rc = reg_rd(R_MAGIC,   &magic))   != EX_OK) return rc;
    if ((rc = reg_rd(R_VERSION, &version)) != EX_OK) return rc;
    if ((rc = reg_rd(R_CLK_MHZ, &clk_mhz)) != EX_OK) return rc;
    if ((rc = reg_rd(R_WIDTH,   &width))   != EX_OK) return rc;
    if ((rc = reg_rd(R_HEIGHT,  &height))  != EX_OK) return rc;
    if ((rc = reg_rd(R_LEN,     &data_len))!= EX_OK) return rc;
    if ((rc = reg_rd(R_CTRL,    &ctrl))    != EX_OK) return rc;
    if ((rc = reg_rd(R_STATUS,  &status))  != EX_OK) return rc;
    if ((rc = reg_rd(R_PTR,     &ptr))     != EX_OK) return rc;
    if ((rc = reg_rd(R_FRAME,   &f_cnt))   != EX_OK) return rc;

    printf("=== 寄存器快照（0x40 MAGIC=0x%08x 自检通过）===\n", (unsigned)magic);
    printf("  VERSION=0x%08x  CLK_MHZ=%u  WIDTH=%u HEIGHT=%u  DATA_LEN=%u\n",
           (unsigned)version, (unsigned)clk_mhz, (unsigned)width,
           (unsigned)height, (unsigned)data_len);
    printf("  CTRL=0x%08x（bit0 enable=%u, bit1 trigger=%u, [3:2] pattern=%u, bit4 cam_sel=%u）\n",
           (unsigned)ctrl, (unsigned)(ctrl & CTRL_ENABLE ? 1u : 0u),
           (unsigned)((ctrl >> 1) & 1u), (unsigned)((ctrl & CTRL_PAT_MASK) >> CTRL_PAT_SHIFT),
           (unsigned)((ctrl & CTRL_CAM_SEL) ? 1u : 0u));
    printf("  STATUS=0x%08x（bit0 ready=%u, bit1 busy=%u）  FRAME_CNT=%u  PTR=%u\n",
           (unsigned)status, (unsigned)(status & STATUS_READY ? 1u : 0u),
           (unsigned)((status >> 1) & 1u), (unsigned)f_cnt, (unsigned)ptr);

    /* ---- 4. 几何与帧缓存布局自检 ---- */
    if (width == 0u || height == 0u || (uint32_t)(width * height) != FB_IMAGE_PIX) {
        fprintf(stderr, "!! 寄存器报的 %ux%u 与本工具的布局假设不符：本工具按 RTL 的\n"
                        "   PTR_BYTES=0x%X（ep_bar_win.v:46）与 640x480 计算像素位置，\n"
                        "   请先确认 FPGA 侧参数（img_frame_src 的 W/H：ph1p_pcie_ep_top.v:591-594）\n",
                (unsigned)width, (unsigned)height, (unsigned)FB_BYTES);
        return EX_DEV;
    }
    const unsigned W = width, H = height;
    const uint32_t pat_bits = ctrl & CTRL_PAT_MASK;   /* 写 CTRL 必须带上 [3:2]，见 ep_bar_win.v:157-161 */

    /* ---- 5. 选源（写 CTRL）---- */
    uint32_t ctrl_new, ctrl_rb;
    if (src_cam) {
        ctrl_new = pat_bits | CTRL_ENABLE | CTRL_CAM_SEL;
        rc = reg_wr(R_CTRL, ctrl_new);
        if (rc != EX_OK)
            return rc;
        if ((rc = reg_rd(R_CTRL, &ctrl_rb)) != EX_OK)
            return rc;
        printf("=== 写源：相机（CTRL ← 0x%08x，回读 0x%08x）===\n",
               (unsigned)ctrl_new, (unsigned)ctrl_rb);
        if (!(ctrl_rb & CTRL_CAM_SEL)) {
            fprintf(stderr, "!! CTRL 回读 bit4=0：cam_sel 没写进去（这条通道是借 "
                            "ctrl_pattern[1] 过去的，ep_bar_win.v:105），不继续\n");
            return EX_DEV;
        }
    } else {
        /* enable=0 + trigger 单拍脉冲：出一帧就静止，避免边写边读撕裂 */
        ctrl_new = pat_bits | CTRL_TRIGGER;
        rc = reg_wr(R_CTRL, ctrl_new);
        if (rc != EX_OK)
            return rc;
        if ((rc = reg_rd(R_CTRL, &ctrl_rb)) != EX_OK)
            return rc;
        printf("=== 写源：测试图源（CTRL ← 0x%08x，回读 0x%08x）===\n",
               (unsigned)ctrl_new, (unsigned)ctrl_rb);
        if (ctrl_rb & CTRL_CAM_SEL) {
            fprintf(stderr, "!! CTRL 回读 bit4=1：cam_sel 没清掉，帧缓存写源还是相机\n");
            return EX_DEV;
        }
    }
    /* trigger 是单拍脉冲（ep_bar_win.v:159-160 写入、:168-170 自动清），回读恒 0 */

    /* ---- 6. 基准帧号（必须在选源之后读：0x4C 的含义由 CTRL[4] 决定，img_frame_src.v:104）---- */
    uint32_t f0;
    if ((rc = read_frame(&f0)) != EX_OK)
        return rc;
    printf("  当前 FRAME_CNT=%u 作为基准\n", (unsigned)f0);

    /* ---- 7. 相机源：先看链路状态，没锁就直接退，不写图 ---- */
    uint32_t cam_a = 0, cam_b = 0, cam_af = 0, cam_bf = 0, cam_ap = 0, cam_bp = 0, cam_i2c = 0;
    if (src_cam) {
        if ((rc = reg_rd(R_CAM_A,        &cam_a))   != EX_OK) return rc;
        if ((rc = reg_rd(R_CAM_B,        &cam_b))   != EX_OK) return rc;
        if ((rc = reg_rd(R_CAM_A_FRAMES, &cam_af))  != EX_OK) return rc;
        if ((rc = reg_rd(R_CAM_B_FRAMES, &cam_bf))  != EX_OK) return rc;
        if ((rc = reg_rd(R_CAM_A_PIX,    &cam_ap))  != EX_OK) return rc;
        if ((rc = reg_rd(R_CAM_B_PIX,    &cam_bp))  != EX_OK) return rc;
        if ((rc = reg_rd(R_CAM_I2C,      &cam_i2c)) != EX_OK) return rc;
        if ((rc = reg_rd(R_STATUS,       &status))  != EX_OK) return rc;   /* cam_sel 已选中，重读 STATUS */

        cam_stat_decode(cam_a, "相机 A", stdout);
        cam_stat_decode(cam_b, "相机 B", stdout);
        printf("  帧缓存的写源是 A 路（ph1p_pcie_ep_top.v:899-917 只接了 w_a_*）\n");
        printf("  A 路 VSYNC 计数=%u 像素组计数=%u | B 路 VSYNC=%u 像素组=%u | "
               "I2C 忙=%u 上电计数=%u\n",
               (unsigned)cam_af, (unsigned)cam_ap, (unsigned)cam_bf, (unsigned)cam_bp,
               (unsigned)(cam_i2c & 1u), (unsigned)(cam_i2c >> 9));

        rc = cam_stat_verdict(cam_a, "相机 A", stderr);
        if (rc != EX_OK) {
            fprintf(stderr, "   ⇒ 不写图、直接退出（退出码 %d）。链路没锁时帧缓存里没有相机数据\n",
                    rc);
            return rc;
        }
        if (!(status & STATUS_READY))
            fprintf(stderr, "!! 相机 A 告警：STATUS.ready=0 → 到目前为止一帧都没写进帧缓存\n");
    }

    /* ---- 8. 等一帧（测试图源会重发 trigger）---- */
    rc = wait_new_frame(f0, timeout_ms, src_cam ? 0 : 1,
                        pat_bits, src_cam ? "相机" : "测试图源");
    if (rc != EX_OK) {
        if (src_cam)
            fprintf(stderr, "   ⇒ 不写图、直接退出（退出码 %d）：帧缓存里没有可用的相机帧\n", rc);
        return rc;
    }

    /* ---- 9. 确认写口空闲 + 记录 dump 前帧号 ----
     * busy=1 时帧缓存正被改写，这张图一定是半截的，所以先等它落下来；
     * 等不到就继续搬但标记 suspect（图照写、退出码 5 + 告警），不静默当成功。 */
    int suspect = 0;
    {
        uint64_t t0 = now_ms();
        for (;;) {
            if ((rc = read_status(&status)) != EX_OK)
                return rc;
            if (!(status & STATUS_BUSY))
                break;
            if (now_ms() - t0 >= timeout_ms) {
                fprintf(stderr, "!! STATUS.busy 连续 %u ms 为 1：帧缓存正在被写，这张图会是"
                                "半截的（退出码 %d）\n", timeout_ms, EX_INCONSISTENT);
                suspect = 1;
                break;
            }
            msleep(POLL_MS);
        }
    }
    uint32_t f_pre;
    if ((rc = read_frame(&f_pre)) != EX_OK)
        return rc;

    /* 搬运整帧之前再自检一次 MAGIC（读 0x40 不影响 PTR） */
    if ((rc = magic_gate("抓帧前（已就绪）")) != EX_OK)
        return rc;

    /* ---- 10. PTR 归零 + 整帧搬运 ---- */
    if ((rc = reg_wr(R_PTR, 0)) != EX_OK)
        return rc;
    if ((rc = reg_rd(R_PTR, &ptr)) != EX_OK)
        return rc;
    if (ptr != 0u) {
        fprintf(stderr, "!! 写 PTR=0 后回读 = %u（不为 0）→ 指针没清干净，不继续\n",
                (unsigned)ptr);
        return EX_DEV;
    }

    uint32_t *fb = malloc(FB_BYTES);
    if (!fb) {
        fprintf(stderr, "!! malloc %u 字节失败\n", (unsigned)FB_BYTES);
        return EX_OUTPUT;
    }
    printf("=== 搬运整帧：%u 字 × 4B（逐 dw 读 0x60，PTR 自动 +4）===\n",
           (unsigned)FB_WORDS);
    uint64_t t_dump0 = now_ms();
    for (size_t i = 0; i < FB_WORDS; i++) {
        rc = data_rd(i, &fb[i]);
        if (rc != EX_OK) {
            free(fb);
            fprintf(stderr, "   ⇒ 已读 %zu/%u 字，帧数据不完整，不写图（退出码 %d）\n",
                    i, (unsigned)FB_WORDS, rc);
            return rc;
        }
    }
    uint64_t dump_ms = now_ms() - t_dump0;

    /* ---- 11. 收尾自检 ---- */
    uint32_t f_post, st_post, len_post;
    if ((rc = magic_gate("搬运后")) != EX_OK) { free(fb); return rc; }
    if ((rc = read_frame(&f_post))  != EX_OK) { free(fb); return rc; }
    if ((rc = read_status(&st_post))!= EX_OK) { free(fb); return rc; }
    if ((rc = reg_rd(R_LEN, &len_post)) != EX_OK) { free(fb); return rc; }
    if ((rc = reg_rd(R_PTR, &ptr))  != EX_OK) { free(fb); return rc; }

    uint32_t frame_word = fb[0];   /* 字 0 = 帧号（img_frame_src.v:14-15/:166-171） */

    /* ---- 12. 拼 PGM：像素 k ← 字 (k/4 + 1) 的第 (k%4) 字节 ---- */
    const size_t npix_img   = (size_t)W * H;
    const size_t npix_store = (FB_PIXELS < npix_img) ? FB_PIXELS : npix_img;
    unsigned char *img = malloc(npix_img);
    if (!img) {
        fprintf(stderr, "!! malloc %zu 字节失败\n", npix_img);
        free(fb);
        return EX_OUTPUT;
    }
    memset(img, 0, npix_img);      /* 最后 4 个像素帧缓存里没有 → 保持 0 */
    fb_to_image(fb, img, npix_img, npix_store);

    FILE *fp = fopen(out_path, "wb");
    if (!fp) {
        fprintf(stderr, "!! 打不开输出文件 %s：%s\n", out_path, strerror(errno));
        free(img); free(fb);
        return EX_OUTPUT;
    }
    if (fprintf(fp, "P5\n%u %u\n255\n", W, H) < 0 ||
        fwrite(img, 1, npix_img, fp) != npix_img || fflush(fp) != 0 || fclose(fp) != 0) {
        fprintf(stderr, "!! 写 %s 失败：%s\n", out_path, strerror(errno));
        free(img); free(fb);
        return EX_OUTPUT;
    }

    /* ---- 13. 一致性校验 ---- */
    int torn = (f_post != f_pre);
    int frame_ok;
    if (src_cam) {
        /* cam_fb_writer.v:271 写 frame_cnt+1、:279 EOF 才 frame_cnt++ ⇒ 字 0 == FRAME_CNT */
        frame_ok = (frame_word == f_pre);
    } else {
        /* img_frame_src.v:170 写 frame_cnt_t、:185 帧末才自增 ⇒ 字 0 == FRAME_CNT - 1 */
        frame_ok = (f_pre == frame_word + 1u);
    }

    printf("\n=== 结果 ===\n");
    printf("  输出文件        : %s（P5 二进制 PGM，%ux%u 灰度，%zu 字节像素）\n",
           out_path, W, H, npix_img);
    printf("  写源            : %s（CTRL[4]=%d）\n", src_cam ? "相机" : "测试图源",
           src_cam ? 1 : 0);
    printf("  帧号（缓存字 0）: %u\n", (unsigned)frame_word);
    printf("  FRAME_CNT       : 抓帧前 %u → 搬运前 %u → 搬运后 %u  %s\n",
           (unsigned)f0, (unsigned)f_pre, (unsigned)f_post,
           torn ? "← 搬运期间变过！" : "（搬运期间未变）");
    printf("  帧号关系        : 字0=%u 与 搬运前 FRAME_CNT=%u %s（%s）\n",
           (unsigned)frame_word, (unsigned)f_pre, frame_ok ? "自洽" : "**对不上**",
           src_cam ? "相机：字0 == FRAME_CNT（cam_fb_writer.v:271/:279）"
                   : "测试图：字0 == FRAME_CNT-1（img_frame_src.v:170/:185）");
    printf("  DATA_LEN        : 抓帧前 %u / 搬运后 %u（期望 %u = %ux%u）\n",
           (unsigned)data_len, (unsigned)len_post, (unsigned)(W * H), W, H);
    if (src_cam)
        printf("                    （相机源的 DATA_LEN 是常量 %u，cam_fb_writer.v:116：\n"
               "                     它不能用来判「有没有出帧」，判帧只看 FRAME_CNT）\n",
               (unsigned)(W * H));
    printf("  STATUS（搬运后）: 0x%08x（bit0 ready=%u, bit1 busy=%u）  PTR（搬运后）=%u\n",
           (unsigned)st_post, (unsigned)(st_post & STATUS_READY ? 1u : 0u),
           (unsigned)((st_post >> 1) & 1u), (unsigned)ptr);
    printf("  读到全 F 次数   : %u（大于 0 会在读到时立即熔断退出，走不到这里）\n", g_allf);
    printf("  搬运耗时        : %llu ms（%.1f MB/s，PIO 单 dw 读）\n",
           (unsigned long long)dump_ms,
           dump_ms ? (double)FB_BYTES / (double)dump_ms / 1000.0 : 0.0);
    printf("  布局说明        : 字 0 = 帧号，像素字只有 %u 个 ⇒ PGM 前 %zu 个像素来自帧缓存，\n",
           (unsigned)(FB_WORDS - 1u), npix_store);
    printf("                    末尾 %zu 个像素（x=%u..%u, y=%u）帧缓存里不存，已填 0\n",
           npix_img - npix_store, (unsigned)(npix_store % W), W - 1u, H - 1u);

    size_t bad = 0;
    if (!src_cam) {
        bad = pattern_check(img, npix_store, W, frame_word,
                            "若整帧不符，先查 RTL 布局/搬运；只末尾几个不符，多半是撕裂");
    }

    printf("  一致性          : %s\n",
           (torn || !frame_ok || suspect)
               ? "**不通过**（图已写出，但请勿当可信帧用；原因见上面的告警）"
               : "OK（搬运期间帧号未变、帧号关系自洽）");

    if (src_cam && torn)
        fprintf(stderr, "!! 搬运期间 FRAME_CNT 变了（%u → %u）：相机在连续出帧，这张图很可能"
                        "是两帧的混合（PIO 带宽 %s，一帧需 %llu ms）。图已写出，请以帧号为准\n",
                (unsigned)f_pre, (unsigned)f_post, "~2.6MB/s", (unsigned long long)dump_ms);
    if (!frame_ok)
        fprintf(stderr, "!! 缓存字 0 的帧号与 FRAME_CNT 关系不符：布局假设（字 0 = 帧号、"
                        "像素从字 1 起）或帧同步需要复核\n");

    free(img);
    free(fb);
    if (torn || !frame_ok || bad || suspect)
        return EX_INCONSISTENT;
    return EX_OK;
}
