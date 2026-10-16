// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动：相机链路 / 推帧的初始化与状态。
 *
 * 为什么要有这一层：FPGA 每次重新配置（下 bit）或主机每次重启之后，EP 里所有
 * 寄存器都归零，链路要人工做三件事才通 ——
 *   ① 写 0x44 CTRL 的使能位（总使能、相机写源、允许整帧搬进 DDR3、允许推帧）；
 *   ② 重新触发一次传感器初始化（CTRL[2] 电平一变，顶层就重走 xclr 拉放 +
 *      201 条配置表，见 ph1p_pcie_ep_top.v 的相机段）；
 *   ③ 把主机收帧缓冲的物理地址写进推帧目标（0xB0/0xB4）、把缓冲内 mailbox 页里的
 *      门铃字地址写进门铃寄存器（0xB8/0xBC）。
 * 这三步以前靠人手工补，本文件把它们做进探测流程：探测时自动做一遍，并把结果
 * 记在 hw 上，用户态再用 ioctl 复核、单步重触发或重设地址。
 *
 * 寄存器表与位定义见 fpga/rtl/ep_side/img/ep_bar_win.v 头部，驱动与用户态共用的
 * 那份拷贝在 include/anlogic_pci_ioctl.h（本文件只用那份常量，不另抄偏移）。
 *
 * 访问规约（与 anlogic_pci_ctrl.c 的 ioctl 路径同一条）：本文件只碰 BAR0，任何一次
 * 读之前先读 0x40 验 MAGIC，读到 0xffffffff 立即熔断返回 -EIO，单笔只做 32 位对齐
 * 访问。写之前也验一次 MAGIC —— 设备已经不响应时不要把控制位写进别的窗口。
 *
 * 失败处理：探测时的初始化失败**不让 probe 失败**（老 FPGA 映像里没有这组寄存器，
 * 硬失败会让驱动装不上），但一定 pr_err 打全，并把 armed 留在 0，用户态可由
 * ANLOGIC_IOCLINKSTATUS 看出卡在哪一步、再用另外两条 ioctl 重试。
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "anlogic_pci.h"

/* 触发之后等链路起来：顶层先拉低 xclr ~134ms，再走完 201 条 I2C 表并等 lane 对齐 */
#define ANLOGIC_LINK_LOCK_POLL_MS   50u
/* 开机/重触发时先等 FPGA 自己的上电初始化把链路带起来（别一脚插进去把它打断） */
#define ANLOGIC_LINK_BOOT_WAIT_MS   500u
/* 重触发之后再等多久（实测本板这里要几秒：lane 先 0x8、中间可能掉回 0、最后 0xf） */
#define ANLOGIC_LINK_RETRIG_WAIT_MS 3000u

/* 探测时读/写一个 BAR0 寄存器。hw->bar[config_bar_idx] 由 hw_open 映射好 */
static void __iomem *anlogic_pci_link_bar0(struct anlogic_pci_hw *hw)
{
    if (!hw || hw->config_bar_idx < 0)
        return NULL;
    if (!hw->bar[hw->config_bar_idx])
        return NULL;
    if (hw->bar_map_len[hw->config_bar_idx] < ANLOGIC_BAR_LINK_WIN_END) {
        pr_err("链路：BAR%d 只映射了 0x%llx 字节，装不下 0x%x 的寄存器窗。\n",
               hw->config_bar_idx,
               (unsigned long long)hw->bar_map_len[hw->config_bar_idx],
               ANLOGIC_BAR_LINK_WIN_END);
        return NULL;
    }

    return hw->bar[hw->config_bar_idx];
}

/*
 * 0x40 MAGIC 门禁。本文件每一次读写之前都调一次：设备被重新配置/拔掉之后 BAR 会
 * 读回全 F 或别的东西，那时候后面的偏移就不再是「本 EP 的寄存器」了。
 */
static int anlogic_pci_link_magic_gate(struct anlogic_pci_hw *hw)
{
    void __iomem *bar0 = anlogic_pci_link_bar0(hw);
    u32 magic;

    if (!bar0)
        return -ENODEV;

    magic = ioread32(bar0 + ANLOGIC_BAR_MAGIC_OFF);
    if (magic == ANLOGIC_MMIO_ALL_F) {
        pr_err("链路：BAR0+0x40 读到 0xffffffff，设备已不可用，熔断。\n");
        return -EIO;
    }
    if (magic != ANLOGIC_BAR_MAGIC_VAL) {
        pr_err("链路：BAR0+0x40 MAGIC 0x%08x != 0x%08x，拒绝访问。\n",
               magic, ANLOGIC_BAR_MAGIC_VAL);
        return -ENODEV;
    }

    return 0;
}

static int anlogic_pci_link_read32(struct anlogic_pci_hw *hw, u32 off, u32 *val)
{
    void __iomem *bar0;
    int rv;

    rv = anlogic_pci_link_magic_gate(hw);
    if (rv < 0)
        return rv;
    bar0 = anlogic_pci_link_bar0(hw);

    *val = ioread32(bar0 + off);
    if (*val == ANLOGIC_MMIO_ALL_F) {
        pr_err("链路：BAR0+0x%x 读到 0xffffffff，设备已不可用，熔断。\n", off);
        return -EIO;
    }

    return 0;
}

static int anlogic_pci_link_write32(struct anlogic_pci_hw *hw, u32 off, u32 val)
{
    void __iomem *bar0;
    int rv;

    rv = anlogic_pci_link_magic_gate(hw);
    if (rv < 0)
        return rv;
    bar0 = anlogic_pci_link_bar0(hw);

    iowrite32(val, bar0 + off);

    return 0;
}

/* 三个 ioctl 与探测流程都要读的相机状态：lane_valid 非 0 = MIPI 链路已锁 */
static int anlogic_pci_link_read_cam(struct anlogic_pci_hw *hw, u32 *stat)
{
    return anlogic_pci_link_read32(hw, ANLOGIC_BAR_CAM_A_STAT_OFF, stat);
}

/*
 * 0x44 CTRL 的读改写。写 CTRL 会把 [3:2] 一起覆盖（ep_bar_win.v 的寄存器写分支），
 * 所以永远先读出来再按位改；bit1（单帧触发）是自清脉冲，一律写 0 免得顺手多出一帧。
 * 返回写进去的值；*rb 给回读值（用于复核写入是否真的生效）。
 */
static int anlogic_pci_link_ctrl_update(struct anlogic_pci_hw *hw, u32 set_bits,
                                       u32 flip_bits, u32 *rb)
{
    u32 ctrl;
    int rv;

    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_CTRL_OFF, &ctrl);
    if (rv < 0)
        return rv;

    ctrl = (ctrl | set_bits) & ~ANLOGIC_CTRL_TRIGGER;
    ctrl ^= flip_bits;

    rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_CTRL_OFF, ctrl);
    if (rv < 0)
        return rv;

    if (rb) {
        rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_CTRL_OFF, rb);
        if (rv < 0)
            return rv;
    }

    hw->link_ctrl = ctrl;

    return 0;
}

/*
 * 等相机链路起来。两个判据，满足一个就返回：
 *   - A 路四条 lane 全对齐（lane_valid == 0xf）—— 「完全就绪」那个状态；
 *   - 帧计数（0x4C）相对进来时的值变了 —— 帧已经在出，链路功能上已经通。
 * 为什么要两个：本板实测重触发之后 lane 是逐步对齐的（先 0x8、中间还会掉回 0、
 * 几秒后才 0xf，见 link_ctl/上板记录），只等 lane_valid 非 0 会把正在恢复的状态
 * 当成已锁定，而帧计数在动才是「真的在出帧」。返回时把两个判据的实际值都带出去。
 */
static int anlogic_pci_link_wait_up(struct anlogic_pci_hw *hw,
                                    unsigned int wait_ms, u32 *cam_stat,
                                    u32 *frame_cnt, unsigned int *frames_flowing,
                                    unsigned int *waited_ms)
{
    unsigned int waited = 0;
    u32 stat = 0, cnt = 0, cnt0 = 0;
    int rv;

    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_FRAME_CNT_OFF, &cnt0);
    if (rv < 0)
        return rv;
    cnt = cnt0;

    for (;;) {
        rv = anlogic_pci_link_read_cam(hw, &stat);
        if (!rv)
            rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_FRAME_CNT_OFF, &cnt);
        if (rv < 0)
            return rv;
        if (ANLOGIC_CAM_LANE_VALID(stat) == ANLOGIC_CAM_LANE_ALL || cnt != cnt0)
            break;
        if (waited >= wait_ms)
            break;
        msleep(ANLOGIC_LINK_LOCK_POLL_MS);
        waited += ANLOGIC_LINK_LOCK_POLL_MS;
    }

    if (cam_stat)
        *cam_stat = stat;
    if (frame_cnt)
        *frame_cnt = cnt;
    if (frames_flowing)
        *frames_flowing = (cnt != cnt0) ? 1u : 0u;
    if (waited_ms)
        *waited_ms = waited;

    return 0;
}

/* 探测成功时那一行状态：链路是否锁、推帧是否已初始化、目标地址 */
static void anlogic_pci_link_print(const struct anlogic_pci_hw *hw, u32 cam_stat,
                                   int armed, u32 pushed, u32 ddr_status,
                                   u32 ddr_seq)
{
    u32 lane = ANLOGIC_CAM_LANE_VALID(cam_stat);

    pr_info("链路状态：相机 %s（A 路 lane_valid=0x%x、状态字 0x%08x）、"
            "推帧 %s、目标 0x%llx、门铃 0x%llx、DDR3 %s（状态字 0x%08x，已落 %u 帧）、"
            "已推 %u 帧。\n",
            lane ? "已锁" : "未锁", lane, cam_stat,
            armed ? "已初始化" : "**未初始化**",
            (unsigned long long)hw->link_push_dst,
            (unsigned long long)hw->link_push_db,
            ((ddr_status & ANLOGIC_DDR_STATUS_ALIVE) &&
             (ddr_status & ANLOGIC_DDR_STATUS_CAL_DONE)) ? "就绪" : "未就绪",
            ddr_status, ddr_seq, pushed);
}

/*
 * 探测时的初始化：MAGIC 门禁 → 推帧/门铃地址 → CTRL 使能位 → 按需重触发 → 打印状态。
 *
 * 为什么地址写在使能位**之前**：使能位一置上，推帧引擎遇到下一帧就按寄存器里当前的
 * 地址推。重装驱动时那几个地址寄存器还是上一次加载留下的旧值（那块缓冲已经还给内核
 * 了），先置使能位就可能把一帧打到已经不属于我们的内存里。地址先写好并回读复核过了，
 * 再置使能位，这个窗口就不存在。
 *
 * 重触发放在最后：它要等 xclr 拉放 + 201 条配置表，等的时候链路已经在出帧，地址与
 * 使能位必须已经就绪。
 *
 * 任何一步失败都返回负值并把原因打出来；调用方（hw_open）把失败当非致命。
 */
int anlogic_pci_link_arm(struct anlogic_pci_hw *hw)
{
    u32 ctrl, cam_stat = 0, ddr_status = 0, ddr_seq = 0, pushed = 0, frame_cnt = 0;
    u32 dst_lo, dst_hi, db_lo, db_hi;
    u64 dst, db;
    unsigned int waited = 0;
    int armed = 0;
    int rv;

    if (!hw || !hw->pdev)
        return -EINVAL;

    /* 收帧缓冲必须先分配好：推帧目标就是它，门铃字也在它里面 */
    if (!hw->frame_virt || !hw->frame_mb) {
        pr_err("链路：收帧缓冲还没分配好，无法初始化推帧（先看收帧缓冲的报错）。\n");
        return -EINVAL;
    }

    rv = anlogic_pci_link_magic_gate(hw);
    if (rv < 0)
        return rv;

    /*
     * ① 推帧目标 = 槽 0 的**像素区起点** = 槽基址 + 槽头字节数（16）。
     *    槽基址那 16 字节由驱动写在看门铃时补的槽头（seq/len/flags）占用，
     *    推帧引擎若从槽基址写起，帧的头 16 字节就会被槽头盖掉；而主机
     *    （frame_grab）取像素的起点是 slot*stride + hdr_bytes，正好也是像素区
     *    起点 —— 两边对上，帧既不会被槽头覆盖，也不会被整块搬移。
     *    推帧引擎不逐帧轮换目标地址，所以推来的帧永远落在槽 0。
     *    门铃地址 = 缓冲内 mailbox 页里的门铃字（推帧引擎每推完一帧往那儿写
     *    本帧的已推帧数，驱动见它变大就补槽头与 mailbox.seq/count）。
     */
    dst = (u64)hw->frame_phys + (u64)hw->frame_hdr_bytes;
    db = (u64)hw->frame_phys + (u64)hw->frame_mb_off +
         (u64)offsetof(struct anlogic_frame_mailbox, push_dbell);
    dst_lo = (u32)dst;
    dst_hi = (u32)(dst >> 32);
    db_lo = (u32)db;
    db_hi = (u32)(db >> 32);

    rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF, dst_lo);
    if (!rv)
        rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DST_HI_OFF, dst_hi);
    if (!rv)
        rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, db_lo);
    if (!rv)
        rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF, db_hi);
    if (rv < 0) {
        pr_err("链路：写推帧目标/门铃地址失败 %d，初始化中断。\n", rv);
        return rv;
    }

    /* 复核两笔：写入没生效（老映像没有这几个口）时要当场看出来，别当成功 */
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF, &ctrl);
    if (rv < 0)
        return rv;
    if (ctrl != dst_lo) {
        pr_err("链路：0xB0 回读 0x%08x != 写入 0x%08x，推帧目标没配好。\n",
               ctrl, dst_lo);
        return -EIO;
    }
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, &ctrl);
    if (rv < 0)
        return rv;
    if (ctrl != db_lo) {
        pr_err("链路：0xB8 回读 0x%08x != 写入 0x%08x，门铃地址没配好。\n",
               ctrl, db_lo);
        return -EIO;
    }
    hw->link_push_dst = dst;
    hw->link_push_db = db;

    /*
     * ② CTRL 使能位：总使能 + 相机写源 + 允许相机帧写进 DDR3 + 允许推帧
     *    （bit7=0 走 DDR3 源，见 include/anlogic_pci_ioctl.h 的 ANLOGIC_CTRL_ARM_BITS）。
     *    bit2 保持当前值不动 —— 它是重触发电平，翻了就等于做一次传感器初始化。
     */
    rv = anlogic_pci_link_ctrl_update(hw, ANLOGIC_CTRL_ARM_BITS, 0, &ctrl);
    if (rv < 0) {
        pr_err("链路：写 0x44 CTRL 失败 %d，初始化中断。\n", rv);
        return rv;
    }
    if ((ctrl & ANLOGIC_CTRL_ARM_BITS) != ANLOGIC_CTRL_ARM_BITS) {
        pr_err("链路：0x44 回读 0x%08x 里使能位没置全（要 0x%08x），初始化中断。\n",
               ctrl, ANLOGIC_CTRL_ARM_BITS);
        return -EIO;
    }
    armed = 1;

    pr_info("链路：推帧目标 0x%llx、门铃 0x%llx（都在收帧缓冲 0x%llx 里），"
            "0x44 CTRL=0x%08x。\n",
            (unsigned long long)dst, (unsigned long long)db,
            (unsigned long long)hw->frame_phys, ctrl);

    /*
     * ③ 按需重触发相机初始化：只在链路完全没锁（lane_valid==0）时做 —— 已经有 lane
     *    对齐说明传感器在出帧，再翻转一次 CTRL[2] 会打断正在跑的采集。
     *    分两步：先等一会看 FPGA 自己的上电初始化（xclr + 201 条表）能不能把它带起来，
     *    还起不来才补一次重触发；重触发之后再等几秒（本板实测 lane 全对齐要几秒）。
     */
    rv = anlogic_pci_link_read_cam(hw, &cam_stat);
    if (rv < 0)
        return rv;
    if (ANLOGIC_CAM_LANE_VALID(cam_stat)) {
        pr_info("链路：A 路 lane_valid=0x%x（共 4 条 lane），不重触发（免得打断采集）。\n",
                ANLOGIC_CAM_LANE_VALID(cam_stat));
    } else {
        unsigned int flowing = 0;

        rv = anlogic_pci_link_wait_up(hw, ANLOGIC_LINK_BOOT_WAIT_MS, &cam_stat,
                                     &frame_cnt, &flowing, &waited);
        if (rv < 0)
            return rv;
        if (ANLOGIC_CAM_LANE_VALID(cam_stat) || flowing) {
            pr_info("链路：上电初始化已经把它带起来了（lane_valid=0x%x、0x4C=%u），不重触发。\n",
                    ANLOGIC_CAM_LANE_VALID(cam_stat), frame_cnt);
        } else {
            pr_info("链路：等了 %u ms 相机仍没起来（lane_valid=0），翻转 CTRL[2] 重新初始化传感器。\n",
                    waited);
            rv = anlogic_pci_link_ctrl_update(hw, ANLOGIC_CTRL_ARM_BITS,
                                              ANLOGIC_CTRL_MCLK_SEL, &ctrl);
            if (rv < 0) {
                pr_err("链路：重触发写 0x44 失败 %d。\n", rv);
                return rv;
            }
            rv = anlogic_pci_link_wait_up(hw, ANLOGIC_LINK_RETRIG_WAIT_MS,
                                         &cam_stat, &frame_cnt, &flowing, &waited);
            if (rv < 0)
                return rv;
            if (ANLOGIC_CAM_LANE_VALID(cam_stat) == ANLOGIC_CAM_LANE_ALL) {
                pr_info("链路：重触发后 %u ms 内四条 lane 全部对齐（lane_valid=0xf）。\n",
                        waited);
            } else if (flowing) {
                pr_info("链路：重触发后 %u ms 内帧已经在出（0x4C=%u），"
                        "lane_valid=0x%x（还没全对齐，链路仍在收尾）。\n",
                        waited, frame_cnt, ANLOGIC_CAM_LANE_VALID(cam_stat));
            } else {
                pr_warn("链路：重触发后等了 %u ms 相机仍未出帧（lane_valid=0x%x、"
                        "状态字 0x%08x）—— 传感器/接线/供电侧的问题，不在这条通路上；"
                        "可用 link_ctl retrig / status 复核。\n",
                        waited, ANLOGIC_CAM_LANE_VALID(cam_stat), cam_stat);
            }
        }
    }

    /* ④ 顺带把 DDR3 与推帧的握手字读一遍，供那行状态与后面的 ioctl 对照 */
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_DDR_STATUS_OFF, &ddr_status);
    if (!rv)
        rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_DDR_SEQ_OFF, &ddr_seq);
    if (!rv)
        rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DONE_OFF, &pushed);
    if (rv < 0)
        return rv;

    hw->link_armed = (unsigned int)armed;
    anlogic_pci_link_print(hw, cam_stat, armed, pushed, ddr_status, ddr_seq);

    return 0;
}

/*
 * 卸载时把推帧使能位关掉：收帧缓冲是 dma_alloc_coherent 分配的系统内存，模块一
 * 卸就把这块内存还给内核了；FPGA 若还在往那个地址 MWr，就是往别人刚拿到的内存里
 * 写数据。使能位只在推帧引擎的帧间空闲拍采样（ddr_frame_pusher.v），所以关掉之后
 * 还要等一会儿，让可能正在飞的那一帧（链路满速约 0.4ms，一帧几十毫秒才来一次）
 * 落完，再去释放缓冲。
 *
 * bit5（DDR3 搬运）不动：那条通路只写 FPGA 板上的 DDR3，不碰主机内存。
 */
void anlogic_pci_link_disarm(struct anlogic_pci_hw *hw)
{
    u32 ctrl;
    int rv;

    if (!hw || !hw->link_armed)
        return;

    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_CTRL_OFF, &ctrl);
    if (rv < 0) {
        pr_warn("链路：卸载时读 0x44 失败 %d（设备可能已经没了），跳过关推帧。\n",
                rv);
        hw->link_armed = 0;
        return;
    }
    rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_CTRL_OFF,
                                  ctrl & ~ANLOGIC_CTRL_PUSH_EN);
    if (rv < 0) {
        pr_warn("链路：卸载时关推帧使能失败 %d。\n", rv);
    } else {
        pr_info("链路：卸载前已关推帧使能（0x44 = 0x%08x），等一帧落地。\n",
                ctrl & ~ANLOGIC_CTRL_PUSH_EN);
        msleep(20);
    }

    hw->link_armed = 0;
}

/* 状态查询：把 BAR0 里与链路/推帧有关的那几个字全读出来，一项读失败就整条失败 */
static long anlogic_pci_link_status(struct anlogic_pci_hw *hw,
                                    struct anlogic_ioc_link_status *st)
{
    struct anlogic_frame_mailbox *mb = hw->frame_mb;
    int rv;

    memset(st, 0, sizeof(*st));

    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_CTRL_OFF, &st->ctrl);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_FRAME_CNT_OFF, &st->frame_cnt);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read_cam(hw, &st->cam_stat);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_DDR_SEQ_OFF, &st->ddr_seq);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_DDR_BASE_OFF, &st->ddr_base);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_DDR_STATUS_OFF, &st->ddr_status);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_DDR_LEN_OFF, &st->ddr_len);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DONE_OFF, &st->pushed);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_STATUS_OFF, &st->push_status);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_ERR_OFF, &st->push_err);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_TLP_OFF, &st->push_tlp_cnt);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF, &st->dst_lo);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_HI_OFF, &st->dst_hi);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, &st->db_lo);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF, &st->db_hi);
    if (rv < 0)
        return rv;

    st->lane_valid = ANLOGIC_CAM_LANE_VALID(st->cam_stat);
    st->cam_locked = st->lane_valid ? 1u : 0u;
    st->ddr_ready = ((st->ddr_status & ANLOGIC_DDR_STATUS_ALIVE) &&
                     (st->ddr_status & ANLOGIC_DDR_STATUS_CAL_DONE)) ? 1u : 0u;
    st->push_en = (st->ctrl & ANLOGIC_CTRL_PUSH_EN) ? 1u : 0u;
    st->push_src = (st->ctrl & ANLOGIC_CTRL_PUSH_SRC) ? 1u : 0u;
    st->armed = hw->link_armed;

    if (mb) {
        st->seq = READ_ONCE(mb->seq);
        dma_rmb();
        st->count = READ_ONCE(mb->count);
    }

    return 0;
}

/*
 * 三条链路 ioctl 的实现。调用方（anlogic_pci_ctrl_ioctl）已经拿住 hw->lock、
 * 且确认过 removed，也做过 access_ok；这里只做 copy_from/to_user 与具体动作。
 */
long anlogic_pci_link_ioctl(struct anlogic_pci_cdev *cdev, unsigned int cmd,
                            void __user *arg)
{
    struct anlogic_pci_hw *hw;
    int rv;

    rv = anlogic_pci_cdev_check(__func__, cdev);
    if (rv < 0)
        return rv;

    hw = cdev->hw;
    if (!hw) {
        pr_err("链路 ioctl：hw 为空。\n");
        return -EINVAL;
    }

    switch (cmd) {
    case ANLOGIC_IOCLINKSTATUS:
        {
            struct anlogic_ioc_link_status st;

            rv = anlogic_pci_link_status(hw, &st);
            if (rv < 0)
                return rv;
            if (copy_to_user((void __user *)arg, &st, sizeof(st))) {
                pr_err("链路 ioctl：copy_to_user(状态) 失败。\n");
                return -EFAULT;
            }
        }
        break;

    case ANLOGIC_IOCLINKRETRIG:
        {
            struct anlogic_ioc_link_retrig rt;
            u32 cam_stat = 0, frame_cnt = 0;
            unsigned int wait_ms, waited = 0, flowing = 0;

            if (copy_from_user((void *)&rt, (void __user *)arg, sizeof(rt))) {
                pr_err("链路 ioctl：copy_from_user(重触发) 失败。\n");
                return -EFAULT;
            }
            wait_ms = rt.wait_ms;
            if (wait_ms > ANLOGIC_LINK_MAX_WAIT_MS) {
                pr_err("链路 ioctl：wait_ms %u 超过上限 %u。\n", wait_ms,
                       ANLOGIC_LINK_MAX_WAIT_MS);
                return -EINVAL;
            }

            /* 链路初始化所需的位重新置上 + CTRL[2] 翻转一次 = 重跑一遍传感器初始化 */
            rv = anlogic_pci_link_ctrl_update(hw, ANLOGIC_CTRL_ARM_BITS,
                                              ANLOGIC_CTRL_MCLK_SEL, &rt.ctrl);
            if (rv < 0) {
                pr_err("链路 ioctl：重触发写 0x44 失败 %d。\n", rv);
                return rv;
            }
            hw->link_ctrl = rt.ctrl;

            rv = anlogic_pci_link_wait_up(hw, wait_ms, &cam_stat, &frame_cnt,
                                         &flowing, &waited);
            if (rv < 0)
                return rv;

            rt.cam_stat = cam_stat;
            rt.lane_valid = ANLOGIC_CAM_LANE_VALID(cam_stat);
            rt.cam_locked = rt.lane_valid ? 1u : 0u;
            rt.frame_cnt = frame_cnt;
            rt.frames_flowing = flowing;
            rt.waited_ms = waited;

            if (copy_to_user((void __user *)arg, &rt, sizeof(rt))) {
                pr_err("链路 ioctl：copy_to_user(重触发) 失败。\n");
                return -EFAULT;
            }
            pr_info("链路：手动重触发完成，等了 %u ms，lane_valid=0x%x、0x4C=%u（帧%s）。\n",
                    waited, rt.lane_valid, frame_cnt,
                    flowing ? "在出" : "没出");
        }
        break;

    case ANLOGIC_IOCPUSHCFG:
        {
            struct anlogic_ioc_push_cfg cfg;
            u32 rb, ctrl;
            unsigned int applied = 0;

            if (copy_from_user((void *)&cfg, (void __user *)arg, sizeof(cfg))) {
                pr_err("链路 ioctl：copy_from_user(推帧地址) 失败。\n");
                return -EFAULT;
            }
            if (cfg.flags & ~(ANLOGIC_PUSH_CFG_SET_DST |
                              ANLOGIC_PUSH_CFG_SET_DB)) {
                pr_err("链路 ioctl：flags 0x%x 有不认识的位。\n", cfg.flags);
                return -EINVAL;
            }

            if (cfg.flags & ANLOGIC_PUSH_CFG_SET_DST) {
                rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF,
                                              cfg.dst_lo);
                if (!rv)
                    rv = anlogic_pci_link_write32(hw,
                                                  ANLOGIC_BAR_PUSH_DST_HI_OFF,
                                                  cfg.dst_hi);
                if (rv < 0) {
                    pr_err("链路 ioctl：写推帧目标失败 %d。\n", rv);
                    return rv;
                }
                rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF,
                                             &rb);
                if (rv < 0)
                    return rv;
                if (rb != cfg.dst_lo) {
                    pr_err("链路 ioctl：0xB0 回读 0x%08x != 写入 0x%08x。\n",
                           rb, cfg.dst_lo);
                    return -EIO;
                }
                /* 只有当两个地址都写进同一片缓冲里，驱动才算「已初始化」 */
                applied |= ANLOGIC_PUSH_CFG_SET_DST;
                hw->link_push_dst = ((u64)cfg.dst_hi << 32) | cfg.dst_lo;
            }

            if (cfg.flags & ANLOGIC_PUSH_CFG_SET_DB) {
                rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF,
                                              cfg.db_lo);
                if (!rv)
                    rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF,
                                                  cfg.db_hi);
                if (rv < 0) {
                    pr_err("链路 ioctl：写门铃地址失败 %d。\n", rv);
                    return rv;
                }
                rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, &rb);
                if (rv < 0)
                    return rv;
                if (rb != cfg.db_lo) {
                    pr_err("链路 ioctl：0xB8 回读 0x%08x != 写入 0x%08x。\n",
                           rb, cfg.db_lo);
                    return -EIO;
                }
                applied |= ANLOGIC_PUSH_CFG_SET_DB;
                hw->link_push_db = ((u64)cfg.db_hi << 32) | cfg.db_lo;
            }

            /* 复核：无论写没写，都把寄存器现值和使能位读回来给用户态 */
            rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF,
                                         &cfg.dst_lo);
            if (!rv)
                rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_HI_OFF,
                                             &cfg.dst_hi);
            if (!rv)
                rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF,
                                             &cfg.db_lo);
            if (!rv)
                rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF,
                                             &cfg.db_hi);
            if (!rv)
                rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_CTRL_OFF, &ctrl);
            if (rv < 0)
                return rv;

            cfg.push_en = (ctrl & ANLOGIC_CTRL_PUSH_EN) ? 1u : 0u;
            cfg.push_src = (ctrl & ANLOGIC_CTRL_PUSH_SRC) ? 1u : 0u;
            cfg.armed = hw->link_armed;
            cfg.applied = applied;

            if (copy_to_user((void __user *)arg, &cfg, sizeof(cfg))) {
                pr_err("链路 ioctl：copy_to_user(推帧地址) 失败。\n");
                return -EFAULT;
            }
            pr_info("链路：推帧目标复核 0x%08x%08x、门铃 0x%08x%08x，"
                    "本次写了 0x%x，push_en=%u。\n",
                    cfg.dst_hi, cfg.dst_lo, cfg.db_hi, cfg.db_lo, applied,
                    cfg.push_en);
        }
        break;

    default:
        pr_err("链路 ioctl：不认识的 cmd 0x%x。\n", cmd);
        return -ENOTTY;
    }

    return 0;
}
