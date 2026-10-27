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
#define pr_fmt(fmt) KBUILD_MODNAME ": [link] " fmt

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "anlogic_pci.h"

/*
 * 上电 ARM 重试：RTL 的 arm_reject 含 (ddr_status[0]==0)，CTRL 刚使能、
 * 相机还在出第一帧时必然被拒一次。只在 0xC8==E_ARM_REJ 上等帧重试。
 */
#define ANLOGIC_LINK_ARM_RETRY 50u    /* 次数 */
#define ANLOGIC_LINK_ARM_RETRY_MS 10u /* 每次间隔 ms，共 500ms */

/* 触发之后等链路起来：顶层先拉低 xclr ~134ms，再走完 201 条 I2C 表并等 lane 对齐 */
#define ANLOGIC_LINK_LOCK_POLL_MS 50u
/* 开机/重触发时先等 FPGA 自己的上电初始化把链路带起来（别一脚插进去把它打断） */
#define ANLOGIC_LINK_BOOT_WAIT_MS 500u
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
		pr_err("BAR%d 只映射了 0x%llx 字节，装不下 0x%x 的寄存器窗。\n", hw->config_bar_idx,
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
		pr_err("BAR0+0x40 读到 0xffffffff，设备已不可用，熔断。\n");
		return -EIO;
	}
	if (magic != ANLOGIC_BAR_MAGIC_VAL) {
		pr_err("BAR0+0x40 MAGIC 0x%08x != 0x%08x，拒绝访问。\n", magic,
		       ANLOGIC_BAR_MAGIC_VAL);
		return -ENODEV;
	}

	return 0;
}

/* BAR0 访问器：link.c 与 rka1.c 共用（rka1.c 也要过 0x40 MAGIC 门禁）。 */
int anlogic_pci_link_read32(struct anlogic_pci_hw *hw, u32 off, u32 *val)
{
	void __iomem *bar0;
	int rv;

	rv = anlogic_pci_link_magic_gate(hw);
	if (rv < 0)
		return rv;
	bar0 = anlogic_pci_link_bar0(hw);

	*val = ioread32(bar0 + off);
	if (*val == ANLOGIC_MMIO_ALL_F) {
		pr_err("BAR0+0x%x 读到 0xffffffff，设备已不可用，熔断。\n", off);
		return -EIO;
	}

	return 0;
}

int anlogic_pci_link_write32(struct anlogic_pci_hw *hw, u32 off, u32 val)
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

/* 三个 ioctl 与探测流程都要读的相机状态：lane_valid 非 0 = MIPI 链路已锁。
 * 读哪一路由 hw->link_cam_b（CTRL[8] 的缓存）决定；调用方在改 CTRL 后必须同步它。
 */
static int anlogic_pci_link_read_cam(struct anlogic_pci_hw *hw, u32 *stat)
{
	u32 off = hw->link_cam_b ? ANLOGIC_BAR_CAM_B_STAT_OFF : ANLOGIC_BAR_CAM_A_STAT_OFF;

	return anlogic_pci_link_read32(hw, off, stat);
}

/*
 * 0x44 CTRL 的读改写。写 CTRL 会把 [3:2] 一起覆盖（ep_bar_win.v 的寄存器写分支），
 * 所以永远先读出来再按位改；bit1（单帧触发）是自清脉冲，一律写 0 免得顺手多出一帧。
 * 返回写进去的值；*rb 给回读值（用于复核写入是否真的生效）。
 */
static int anlogic_pci_link_ctrl_update(struct anlogic_pci_hw *hw, u32 set_bits, u32 flip_bits,
					u32 *rb)
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

	return 0;
}

/*
 * 等相机链路起来。两个判据，满足一个就返回：
 *   - 当前选中路（CTRL[8]）四条 lane 全对齐（lane_valid == 0xf）—— 「完全就绪」；
 *   - 帧计数（0x4C）相对进来时的值变了 —— 帧已经在出，链路功能上已经通。
 * 为什么要两个：本板实测重触发之后 lane 是逐步对齐的（先 0x8、中间还会掉回 0、
 * 几秒后才 0xf，见 link_ctl/上板记录），只等 lane_valid 非 0 会把正在恢复的状态
 * 当成已锁定，而帧计数在动才是「真的在出帧」。返回时把两个判据的实际值都带出去。
 */
static int anlogic_pci_link_wait_up(struct anlogic_pci_hw *hw, unsigned int wait_ms, u32 *cam_stat,
				    u32 *frame_cnt, unsigned int *frames_flowing,
				    unsigned int *waited_ms)
{
	ktime_t start = ktime_get();
	s64 waited;
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
		waited = ktime_ms_delta(ktime_get(), start);
		if (waited >= wait_ms)
			break;
		msleep(ANLOGIC_LINK_LOCK_POLL_MS);
	}

	if (cam_stat)
		*cam_stat = stat;
	if (frame_cnt)
		*frame_cnt = cnt;
	if (frames_flowing)
		*frames_flowing = (cnt != cnt0) ? 1u : 0u;
	if (waited_ms)
		*waited_ms = (unsigned int)ktime_ms_delta(ktime_get(), start);

	return 0;
}

/* 探测成功时那一行状态：链路是否锁、推帧是否已初始化、目标地址 */
static void anlogic_pci_link_print(const struct anlogic_pci_hw *hw, u32 cam_stat, int armed,
				   u32 pushed, u32 ddr_status, u32 ddr_seq)
{
	u32 lane = ANLOGIC_CAM_LANE_VALID(cam_stat);

	pr_info("状态：相机 %s（%s 路 lane_valid=0x%x、状态字 0x%08x）\n"
		"推帧 %s、目标 0x%llx、门铃 0x%llx\n"
		"DDR3 %s（状态字 0x%08x，已落 %u 帧）、已推 %u 帧。\n",
	    lane ? "已锁" : "未锁", hw->link_cam_b ? "B" : "A", lane, cam_stat,
	    armed ? "已初始化" : "**未初始化**", (unsigned long long)hw->link_push_dst,
	    (unsigned long long)hw->link_push_db,
	    ((ddr_status & ANLOGIC_DDR_STATUS_ALIVE) && (ddr_status & ANLOGIC_DDR_STATUS_CAL_DONE))
		? "就绪"
		: "未就绪",
	    ddr_status, ddr_seq, pushed);
}

/*
 * link_arm 在 CTRL 使能位置上之后失败时的统一收尾：把 FPGA 收回安全状态再返回错误。
 * 不这样做的话，hw->link_armed 会停在 0，hw_close 里的 disarm 就会跳过，缓冲被释放
 * 时 FPGA 仍可能带着 PUSH_EN/ARM 往这块内存写。
 */
static int anlogic_pci_link_arm_fail(struct anlogic_pci_hw *hw, int rv)
{
	pr_err("初始化在使能后失败（%d），先收回推帧再返回。\n", rv);
	anlogic_pci_link_disarm(hw);
	return rv;
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
	u32 ctrl, rb, rb2, cam_stat = 0, ddr_status = 0, ddr_seq = 0, pushed = 0;
	u32 frame_cnt = 0;
	u32 tx_st = 0, perr = 0;
	u32 dst_lo, dst_hi, db_lo, db_hi;
	u64 dst, db;
	unsigned int waited = 0;
	int rv, try;

	if (!hw || !hw->pdev)
		return -EINVAL;

	/* 收帧缓冲必须先分配好：推帧目标就是它，门铃字也在它里面 */
	if (!hw->frame_virt || !hw->frame_mb) {
		pr_err("收帧缓冲还没分配好，无法初始化推帧（先看收帧缓冲的报错）。\n");
		return -EINVAL;
	}

	rv = anlogic_pci_link_magic_gate(hw);
	if (rv < 0)
		return rv;

	/*
	 * T07: fail closed before touching any push-related register.  Probe the
	 * RKA1 extension first; a legacy image (0x118 reads DEAD_BEEF) returns
	 * -EOPNOTSUPP and the link stays in capture/diagnostic mode only, with the
	 * pusher never armed.
	 */
	rv = anlogic_pci_rka1_probe(hw);
	if (rv < 0) {
		pr_err("FPGA 没有 RKA1 安全扩展（fail closed），推帧不授权；\n"
		       "采集/DDR 诊断仍可用。\n");
		return rv;
	}

	/*
	 * 上一次运行可能把 FPGA 留在 busy/held（进程没 ACK 就退出、模块重装、
	 * 上一个驱动版本留下的状态）。活动期写授权配置会被 RTL 拒（cfg_reject，
	 * ioctl 侧表现为 -EBUSY），所以先把状态收回 IDLE 再配置。
	 * CANCEL 是特权撤销：已开始的包补完整、不发成功门铃、排空后才返回，
	 * 不会把半包写进主机内存。
	 */
	rv = anlogic_pci_rka1_status(hw, &tx_st);
	if (rv < 0) {
		pr_err("读 RKA1 TX_STATUS 失败 %d。\n", rv);
		return rv;
	}
	if (tx_st & (ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY | ANLOGIC_RKA1_ST_HELD |
		     ANLOGIC_RKA1_ST_DRAINING)) {
		pr_info("FPGA 还停在 TX_STATUS=0x%08x（上一次没 ACK 就结束了），\n"
			"先 CANCEL 收回 IDLE。\n",
			tx_st);
		rv = anlogic_pci_rka1_cancel(hw);
		if (rv < 0) {
			pr_err("CANCEL 失败 %d，推帧不授权。\n", rv);
			return rv;
		}
	}

	/*
	 * T07: authorise the DMA once, up front: capacity = the slot data region
	 * and a fresh non-zero session id.  The FPGA validates both again at ARM
	 * time, so a wrong value here cannot cause a transfer.
	 */
	hw->rka1_session = (u32)jiffies | 0x524b0000u; /* 'RK' + low bits of time */
	if (!hw->rka1_session)
		hw->rka1_session = 0x524b0001u;
	rv = anlogic_pci_rka1_configure(hw, hw->frame_slot_bytes, hw->rka1_session);
	if (rv < 0) {
		pr_err("RKA1 授权配置失败 %d，推帧不授权。\n", rv);
		return rv;
	}

	/*
	 * ① 推帧目标 = 槽 0 的**传输数据起点** = 槽基址 + hdr_bytes（128 = 16B 槽头
	 *    + 112B 零 pad）。槽头那 16 字节由驱动在补门铃时写（seq/len/flags），
	 *    128.. 起才是 R10F 头；若从槽基址写起，槽头会被 R10F 头盖掉。而主机
	 *    （frame_grab）取内容的起点是 slot*stride + hdr_bytes，两边对上。
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
		pr_err("写推帧目标/门铃地址失败 %d，初始化中断。\n", rv);
		return rv;
	}

	/* 复核两笔：写入没生效（老映像没有这几个口）时要当场看出来，别当成功 */
	rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF, &rb);
	if (!rv)
		rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_HI_OFF, &rb2);
	if (rv < 0)
		return rv;
	if (rb != dst_lo || rb2 != dst_hi) {
		pr_err("0xB0/0xB4 回读 0x%08x/%08x != 写入 0x%08x/%08x，推帧目标没配好。\n", rb,
		       rb2, dst_lo, dst_hi);
		return -EIO;
	}
	rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, &rb);
	if (!rv)
		rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF, &rb2);
	if (rv < 0)
		return rv;
	if (rb != db_lo || rb2 != db_hi) {
		pr_err("0xB8/0xBC 回读 0x%08x/%08x != 写入 0x%08x/%08x，门铃地址没配好。\n", rb,
		       rb2, db_lo, db_hi);
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
		pr_err("写 0x44 CTRL 失败 %d，初始化中断。\n", rv);
		return rv;
	}
	if ((ctrl & ANLOGIC_CTRL_ARM_BITS) != ANLOGIC_CTRL_ARM_BITS) {
		pr_err("0x44 回读 0x%08x 里使能位没置全（要 0x%08x），初始化中断。\n", ctrl,
		       ANLOGIC_CTRL_ARM_BITS);
		return -EIO;
	}
	/*
	 * 使能位一置上，FPGA 就可能按当前地址发包；从这里开始必须让 disarm 负责收尾
	 * （即使后面的 ARM/状态读取失败），否则卸载时会把仍可 DMA 的缓冲还给内核。
	 */
	hw->link_cam_b = (ctrl & ANLOGIC_CTRL_CAM_B) ? 1u : 0u;
	hw->link_armed = 1;

	/*
	 * ②b T07：one-shot 授权。CTRL 只是模式使能，真正发包靠 ARM_ONE_SHOT：
	 *     一个额度最多完成一帧，推完 FPGA 进 held，等用户态核对完 R10F 再 ACK，
	 *     之后要发下一帧必须重新 ARM（见 anlogic_pci_rka1.c / rka1 ioctl）。
	 */
	/*
	 * 上电时这里几乎一定会先被拒一次：RTL 的 arm_reject 含
	 * (ddr_status[0]==0)——DDR 里还没有可用源帧就直接拒。CTRL 刚置使能、
	 * 传感器还在出第一帧，所以这次拒绝是「还没到时候」而不是「配置错了」。
	 * 因此在 E_ARM_REJ 上按短间隔等第一帧（最多 500ms），其它错误码立刻失败。
	 */
	for (try = 0;; try++) {
		rv = anlogic_pci_rka1_arm_one_shot(hw);
		if (rv >= 0)
			break;
		if (rv != -EINVAL || try >= ANLOGIC_LINK_ARM_RETRY ||
		    anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_ERR_OFF, &perr) < 0 ||
		    perr != ANLOGIC_PUSH_ERR_ARM_REJ) {
			pr_err("RKA1 ARM 失败 %d（容量/会话/链路/MPS 任一项不满足都会拒）。", rv);
			return anlogic_pci_link_arm_fail(hw, rv);
		}
		if (try == 0)
			pr_info("相机还在出第一帧，等它出帧后重试 ARM（最多 %dms）。",
				ANLOGIC_LINK_ARM_RETRY * ANLOGIC_LINK_ARM_RETRY_MS);
		usleep_range(ANLOGIC_LINK_ARM_RETRY_MS * 1000,
			     ANLOGIC_LINK_ARM_RETRY_MS * 1000 + 2000);
	}

	pr_info("推帧目标 0x%llx、门铃 0x%llx（都在收帧缓冲 0x%llx 里），\n"
		"0x44 CTRL=0x%08x。\n",
		(unsigned long long)dst, (unsigned long long)db, (unsigned long long)hw->frame_phys,
		ctrl);

	/*
	 * ③ 按需重触发相机初始化：只在链路完全没锁（lane_valid==0）时做 —— 已经有 lane
	 *    对齐说明传感器在出帧，再翻转一次 CTRL[2] 会打断正在跑的采集。
	 *    分两步：先等一会看 FPGA 自己的上电初始化（xclr + 201 条表）能不能把它带起来，
	 *    还起不来才补一次重触发；重触发之后再等几秒（本板实测 lane 全对齐要几秒）。
	 */
	rv = anlogic_pci_link_read_cam(hw, &cam_stat);
	if (rv < 0)
		return anlogic_pci_link_arm_fail(hw, rv);
	if (ANLOGIC_CAM_LANE_VALID(cam_stat)) {
		pr_info("%s 路 lane_valid=0x%x（共 4 条 lane），不重触发（免得打断采集）。\n",
			hw->link_cam_b ? "B" : "A", ANLOGIC_CAM_LANE_VALID(cam_stat));
	} else {
		unsigned int flowing = 0;

		rv = anlogic_pci_link_wait_up(hw, ANLOGIC_LINK_BOOT_WAIT_MS, &cam_stat, &frame_cnt,
					      &flowing, &waited);
		if (rv < 0)
			return anlogic_pci_link_arm_fail(hw, rv);
		if (ANLOGIC_CAM_LANE_VALID(cam_stat) || flowing) {
			pr_info(
			    "上电初始化已经把它带起来了（lane_valid=0x%x、0x4C=%u），不重触发。\n",
			    ANLOGIC_CAM_LANE_VALID(cam_stat), frame_cnt);
		} else {
			pr_info("等了 %u ms 相机仍没起来（lane_valid=0），\n"
				"翻转 CTRL[2] 重新初始化传感器。\n",
				waited);
			rv = anlogic_pci_link_ctrl_update(hw, ANLOGIC_CTRL_ARM_BITS,
							  ANLOGIC_CTRL_MCLK_SEL, &ctrl);
			if (rv < 0) {
				pr_err("重触发写 0x44 失败 %d。\n", rv);
				return anlogic_pci_link_arm_fail(hw, rv);
			}
			hw->link_cam_b = (ctrl & ANLOGIC_CTRL_CAM_B) ? 1u : 0u;
			rv = anlogic_pci_link_wait_up(hw, ANLOGIC_LINK_RETRIG_WAIT_MS, &cam_stat,
						      &frame_cnt, &flowing, &waited);
			if (rv < 0)
				return anlogic_pci_link_arm_fail(hw, rv);
			if (ANLOGIC_CAM_LANE_VALID(cam_stat) == ANLOGIC_CAM_LANE_ALL) {
				pr_info("重触发后 %u ms 内四条 lane 全部对齐（lane_valid=0xf）。\n",
					waited);
			} else if (flowing) {
				pr_info("重触发后 %u ms 内帧已经在出（0x4C=%u），\n"
					"lane_valid=0x%x（还没全对齐，链路仍在收尾）。\n",
					waited, frame_cnt, ANLOGIC_CAM_LANE_VALID(cam_stat));
			} else {
				pr_warn("重触发后等了 %u ms 相机仍未出帧（lane_valid=0x%x）\n"
					"状态字 0x%08x —— 传感器/接线/供电侧的问题，\n"
					"不在这条通路上；可用 link_ctl retrig / status 复核。\n",
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
		return anlogic_pci_link_arm_fail(hw, rv);

	anlogic_pci_link_print(hw, cam_stat, hw->link_armed, pushed, ddr_status, ddr_seq);

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
	u32 ctrl = 0, push_status = 0;
	int rv, crv = 0;
	int i;

	if (!hw)
		return;

	/*
	 * 不再以 link_armed 作唯一门槛：link_arm 在 CTRL 使能位之后失败时，
	 * link_armed 可能还没置上（或已被 AER 清掉），但 FPGA 仍可能带着 PUSH_EN。
	 * 只要收帧缓冲还在，就做一次收尾；没有缓冲的探针失败路径直接返回。
	 */
	if (!hw->frame_virt) {
		hw->link_armed = 0;
		return;
	}

	/*
	 * T07: cancel first.  CANCEL completes any TLP that was already opened
	 * (never truncates a packet), sends no success doorbell and drains the
	 * source; the helper only returns once busy/draining/held/armed are all
	 * clear.  Only then is it safe to hand the coherent buffer back.
	 * 单次 CANCEL 在慢链路上可能刚好错过，重试几次再放弃；放弃时也必须
	 * 先关 PUSH_EN 并等 PUSH_STATUS.busy 清掉。
	 */
	if (hw->rka1_ext_ok) {
		for (i = 0; i < 5; i++) {
			crv = anlogic_pci_rka1_cancel(hw);
			if (!crv)
				break;
			msleep(20);
		}
		if (crv < 0)
			pr_err(
			    "收尾时 RKA1 CANCEL 重试 5 次仍未确认（%d），继续关模式位并等 busy。\n",
			    crv);
	}

	rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_CTRL_OFF, &ctrl);
	if (rv < 0) {
		pr_warn("收尾时读 0x44 失败 %d（设备可能已经没了），跳过关推帧。\n", rv);
		hw->link_armed = 0;
		return;
	}
	rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_CTRL_OFF, ctrl & ~ANLOGIC_CTRL_PUSH_EN);
	if (rv < 0) {
		pr_warn("收尾时关推帧使能失败 %d。\n", rv);
	} else {
		pr_info("收尾已关推帧使能（0x44 = 0x%08x），等最后一帧落地。\n",
			ctrl & ~ANLOGIC_CTRL_PUSH_EN);
		/* 0xC4.bit0 = 推帧 busy；最多等 50 ms（一帧 MWr 满速约 0.4 ms） */
		for (i = 0; i < 10; i++) {
			if (anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_STATUS_OFF, &push_status) <
			    0)
				break;
			if (!(push_status & ANLOGIC_PUSH_STATUS_BUSY))
				break;
			usleep_range(5000, 6000);
		}
		/*
		 * 关 PUSH_EN 与上面第一次 CANCEL 之间仍可能落下一帧并进 HELD。
		 * HELD 不再写主机内存，但会一直占着槽；再 CANCEL 一次把状态收回 IDLE，
		 * 免得下一次 probe/重新授权还要先处理一个上一轮遗留的 held。
		 */
		if (hw->rka1_ext_ok) {
			u32 st = 0;

			if (!anlogic_pci_rka1_status(hw, &st) &&
			    (st & (ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY |
				   ANLOGIC_RKA1_ST_HELD | ANLOGIC_RKA1_ST_DRAINING))) {
				pr_warn("关推帧后 TX_STATUS=0x%08x 仍有活动，再 CANCEL 一次。\n",
					st);
				if (anlogic_pci_rka1_cancel(hw) < 0)
					pr_err("收尾最终 CANCEL 失败，状态可能仍非 IDLE。\n");
			}
		}
	}

	hw->link_armed = 0;
	hw->link_cam_b = 0;
}

/* 状态查询：把 BAR0 里与链路/推帧有关的那几个字全读出来，一项读失败就整条失败 */
static long anlogic_pci_link_status(struct anlogic_pci_hw *hw, struct anlogic_ioc_link_status *st)
{
	struct anlogic_frame_mailbox *mb = hw->frame_mb;
	int rv;

	memset(st, 0, sizeof(*st));

	rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_CTRL_OFF, &st->ctrl);
	if (rv < 0)
		return rv;
	/* 先按 CTRL[8] 选路，cam_stat/lane_valid 报的就是当前推帧用的那一路 */
	hw->link_cam_b = (st->ctrl & ANLOGIC_CTRL_CAM_B) ? 1u : 0u;
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
			 (st->ddr_status & ANLOGIC_DDR_STATUS_CAL_DONE))
			    ? 1u
			    : 0u;
	st->push_en = (st->ctrl & ANLOGIC_CTRL_PUSH_EN) ? 1u : 0u;
	st->push_src = (st->ctrl & ANLOGIC_CTRL_PUSH_SRC) ? 1u : 0u;
	st->armed = hw->link_armed;

	if (mb) {
		/* 先看门铃补齐，否则只跑 link_ctl status 时 seq/count 会一直停在旧值 */
		anlogic_pci_frame_push_sync(hw);
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
long anlogic_pci_link_ioctl(struct anlogic_pci_cdev *cdev, unsigned int cmd, void __user *arg)
{
	struct anlogic_pci_hw *hw;
	int rv;

	rv = anlogic_pci_cdev_check(__func__, cdev);
	if (rv < 0)
		return rv;

	hw = cdev->hw;
	if (!hw) {
		pr_err("ioctl: hw 为空。\n");
		return -EINVAL;
	}

	switch (cmd) {
	case ANLOGIC_IOCLINKSTATUS: {
		struct anlogic_ioc_link_status st;

		rv = anlogic_pci_link_status(hw, &st);
		if (rv < 0)
			return rv;
		if (copy_to_user((void __user *)arg, &st, sizeof(st))) {
			pr_debug("ioctl: copy_to_user(状态) 失败。\n");
			return -EFAULT;
		}
	} break;

	case ANLOGIC_IOCLINKRETRIG: {
		struct anlogic_ioc_link_retrig rt;
		u32 cam_stat = 0, frame_cnt = 0;
		unsigned int wait_ms, waited = 0, flowing = 0;

		if (copy_from_user((void *)&rt, (void __user *)arg, sizeof(rt))) {
			pr_debug("ioctl: copy_from_user(重触发) 失败。\n");
			return -EFAULT;
		}
		wait_ms = rt.wait_ms;
		if (wait_ms > ANLOGIC_LINK_MAX_WAIT_MS) {
			pr_debug("ioctl: wait_ms %u 超过上限 %u。\n", wait_ms,
				 ANLOGIC_LINK_MAX_WAIT_MS);
			return -EINVAL;
		}

		/* 链路初始化所需的位重新置上 + CTRL[2] 翻转一次 = 重跑一遍传感器初始化 */
		rv = anlogic_pci_link_ctrl_update(hw, ANLOGIC_CTRL_ARM_BITS, ANLOGIC_CTRL_MCLK_SEL,
						  &rt.ctrl);
		if (rv < 0) {
			pr_err("ioctl: 重触发写 0x44 失败 %d。\n", rv);
			return rv;
		}
		hw->link_cam_b = (rt.ctrl & ANLOGIC_CTRL_CAM_B) ? 1u : 0u;

		rv =
		    anlogic_pci_link_wait_up(hw, wait_ms, &cam_stat, &frame_cnt, &flowing, &waited);
		if (rv < 0)
			return rv;

		rt.cam_stat = cam_stat;
		rt.lane_valid = ANLOGIC_CAM_LANE_VALID(cam_stat);
		rt.cam_locked = rt.lane_valid ? 1u : 0u;
		rt.frame_cnt = frame_cnt;
		rt.frames_flowing = flowing;
		rt.waited_ms = waited;

		if (copy_to_user((void __user *)arg, &rt, sizeof(rt))) {
			pr_debug("ioctl: copy_to_user(重触发) 失败。\n");
			return -EFAULT;
		}
		pr_debug("手动重触发完成，等了 %u ms，lane_valid=0x%x、0x4C=%u（帧%s）。\n", waited,
			 rt.lane_valid, frame_cnt, flowing ? "在出" : "没出");
	} break;

	case ANLOGIC_IOCPUSHCFG: {
		struct anlogic_ioc_push_cfg cfg;
		u32 rb, rb2, ctrl;
		unsigned int applied = 0;

		if (copy_from_user((void *)&cfg, (void __user *)arg, sizeof(cfg))) {
			pr_debug("ioctl: copy_from_user(推帧地址) 失败。\n");
			return -EFAULT;
		}
		if (cfg.flags & ~(ANLOGIC_PUSH_CFG_SET_DST | ANLOGIC_PUSH_CFG_SET_DB)) {
			pr_debug("ioctl: flags 0x%x 有不认识的位。\n", cfg.flags);
			return -EINVAL;
		}

		/*
		 * T07: never let a caller point the DMA at an arbitrary address.
		 * Both addresses must land inside the authorised coherent
		 * allocation (frame_phys .. frame_phys+frame_size) and the doorbell
		 * must be the mailbox push_dbell word.  Also refuse while the
		 * one-shot engine is active: the RTL rejects active-config writes
		 * (0xC8=10) and we do not want a half-applied config either.
		 */
		{
			u32 tx_st = 0;
			u64 phys = hw->frame_phys;
			u64 end = phys + hw->frame_size;
			u64 want_dst, want_db;
			u64 exp_dst, exp_db;

			if (!hw->frame_virt || !hw->frame_slot_bytes) {
				pr_err("ioctl: 收帧缓冲未分配，拒绝配置地址。\n");
				return -EINVAL;
			}

			rv = anlogic_pci_rka1_status(hw, &tx_st);
			if (rv < 0)
				return rv;
			if (tx_st & (ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY |
				     ANLOGIC_RKA1_ST_HELD | ANLOGIC_RKA1_ST_DRAINING)) {
				pr_debug("ioctl: 推帧活动期间（TX_STATUS=0x%08x）拒绝改地址。\n",
					 tx_st);
				return -EBUSY;
			}
			want_dst = ((u64)cfg.dst_hi << 32) | cfg.dst_lo;
			want_db = ((u64)cfg.db_hi << 32) | cfg.db_lo;
			/*
			 * 只有一个合法目标：槽 0 的 R10F 数据起点，和 mailbox 里的
			 * push_dbell 字。允许“任意落在缓冲内”的旧实现会让门铃指到
			 * 帧数据上，也会让驱动看不到门铃；这里收紧到逐字节相等。
			 */
			exp_dst = phys + hw->frame_hdr_bytes;
			exp_db = phys + hw->frame_mb_off +
				 offsetof(struct anlogic_frame_mailbox, push_dbell);
			if (exp_dst + hw->frame_slot_bytes > end || exp_db + sizeof(u32) > end) {
				pr_err("ioctl: 驱动记录的槽 0/门铃越出缓冲。\n");
				return -EINVAL;
			}
			if ((cfg.flags & ANLOGIC_PUSH_CFG_SET_DST) && want_dst != exp_dst) {
				pr_debug("ioctl: 目标 0x%llx != 槽 0 数据起点 0x%llx，拒绝。\n",
					 (unsigned long long)want_dst, (unsigned long long)exp_dst);
				return -EINVAL;
			}
			if ((cfg.flags & ANLOGIC_PUSH_CFG_SET_DB) && want_db != exp_db) {
				pr_debug(
				    "ioctl: 门铃 0x%llx != mailbox push_dbell 0x%llx，拒绝。\n",
				    (unsigned long long)want_db, (unsigned long long)exp_db);
				return -EINVAL;
			}
		}

		if (cfg.flags & ANLOGIC_PUSH_CFG_SET_DST) {
			rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF, cfg.dst_lo);
			if (!rv)
				rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DST_HI_OFF,
							      cfg.dst_hi);
			if (rv < 0) {
				pr_err("ioctl: 写推帧目标失败 %d。\n", rv);
				return rv;
			}
			rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF, &rb);
			if (!rv)
				rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_HI_OFF, &rb2);
			if (rv < 0)
				return rv;
			if (rb != cfg.dst_lo || rb2 != cfg.dst_hi) {
				pr_err("ioctl: 0xB0/0xB4 回读 0x%08x/%08x != 写入 0x%08x/%08x。\n",
				       rb, rb2, cfg.dst_lo, cfg.dst_hi);
				return -EIO;
			}
			/* 只有当两个地址都写进同一片缓冲里，驱动才算「已初始化」 */
			applied |= ANLOGIC_PUSH_CFG_SET_DST;
			hw->link_push_dst = ((u64)cfg.dst_hi << 32) | cfg.dst_lo;
		}

		if (cfg.flags & ANLOGIC_PUSH_CFG_SET_DB) {
			rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, cfg.db_lo);
			if (!rv)
				rv = anlogic_pci_link_write32(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF,
							      cfg.db_hi);
			if (rv < 0) {
				pr_err("ioctl: 写门铃地址失败 %d。\n", rv);
				return rv;
			}
			rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, &rb);
			if (!rv)
				rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF, &rb2);
			if (rv < 0)
				return rv;
			if (rb != cfg.db_lo || rb2 != cfg.db_hi) {
				pr_err("ioctl: 0xB8/0xBC 回读 0x%08x/%08x != 写入 0x%08x/%08x。\n",
				       rb, rb2, cfg.db_lo, cfg.db_hi);
				return -EIO;
			}
			applied |= ANLOGIC_PUSH_CFG_SET_DB;
			hw->link_push_db = ((u64)cfg.db_hi << 32) | cfg.db_lo;
		}

		/* 复核：无论写没写，都把寄存器现值和使能位读回来给用户态 */
		rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF, &cfg.dst_lo);
		if (!rv)
			rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DST_HI_OFF, &cfg.dst_hi);
		if (!rv)
			rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, &cfg.db_lo);
		if (!rv)
			rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF, &cfg.db_hi);
		if (!rv)
			rv = anlogic_pci_link_read32(hw, ANLOGIC_BAR_CTRL_OFF, &ctrl);
		if (rv < 0)
			return rv;

		cfg.push_en = (ctrl & ANLOGIC_CTRL_PUSH_EN) ? 1u : 0u;
		cfg.push_src = (ctrl & ANLOGIC_CTRL_PUSH_SRC) ? 1u : 0u;
		/*
		 * 手工修复后同步软件就绪状态：只有 push_en=1 且两个寄存器逐字节
		 * 等于驱动分配的槽 0/门铃时才算 armed，避免 link_ctl push 之后
		 * 仍报“没初始化”，也避免把 0xB0/0xB8 的旧值当成可用目标。
		 */
		if (cfg.push_en && hw->frame_virt && hw->frame_slot_bytes &&
		    (((u64)cfg.dst_hi << 32) | cfg.dst_lo) ==
			(u64)hw->frame_phys + hw->frame_hdr_bytes &&
		    (((u64)cfg.db_hi << 32) | cfg.db_lo) ==
			(u64)hw->frame_phys + hw->frame_mb_off +
			    offsetof(struct anlogic_frame_mailbox, push_dbell))
			hw->link_armed = 1;
		else
			hw->link_armed = 0;
		cfg.armed = hw->link_armed;
		cfg.applied = applied;

		if (copy_to_user((void __user *)arg, &cfg, sizeof(cfg))) {
			pr_debug("ioctl: copy_to_user(推帧地址) 失败。\n");
			return -EFAULT;
		}
		pr_debug("推帧目标复核 0x%08x%08x、门铃 0x%08x%08x，\n"
			 "本次写了 0x%x，push_en=%u。\n",
			 cfg.dst_hi, cfg.dst_lo, cfg.db_hi, cfg.db_lo, applied, cfg.push_en);
	} break;

	default:
		pr_debug("ioctl: 不认识的 cmd 0x%x。\n", cmd);
		return -ENOTTY;
	}

	return 0;
}
