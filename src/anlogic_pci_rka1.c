/* SPDX-License-Identifier: GPL-2.0 */
/*
 * anlogic_pci_rka1.c — T07.rk-align-backend：RKA1 安全扩展（0x100..0x11C）驱动侧
 *
 * 设计边界（与 FPGA 侧 contract/registers.json + ddr_frame_pusher.v 的安全门逐一对应）：
 *   1) 默认不发：FPGA 侧只认 one-shot ARM；CTRL 只置 push 模式位时一个字节都不会推。
 *      本模块把旧「写 CTRL 就连续推帧」的路径彻底拿掉，所有发包都走
 *      anlogic_pci_rka1_arm_one_shot()。
 *   2) fail closed：任何操作前先探测 EXT_MAGIC == "RKA1" 且 EXT_VERSION.major == 1；
 *      旧映像在 0x118 读回 DEAD_BEEF ⇒ 探测失败，一切 RKA1 操作返回 -EOPNOTSUPP，
 *      上层不得回退到旧行为（旧路径已不存在）。
 *   3) 单槽 one-shot：ARM 成功给一帧额度；帧推完 FPGA 进 HELD（TX_STATUS.bit2），
 *      此时源的缓冲被 RTL 锁住不被写者覆盖；用户态核对完整 R10F 后写 ACK_RELEASE
 *      （ACK_SEQ/ACK_SESSION 必须与 held 值逐位相等），释放后必须**再次 ARM**。
 *   4) 生命周期：卸载/离线先 CANCEL，再等 TX_STATUS 的 busy/draining 清零，最后才
 *      允许释放收帧内存；链路重建必须换新 session，旧门铃/ACK 一律不认。
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>

#include "anlogic_pci.h"
#include "anlogic_pci_ioctl.h"

/* RTL 里私有寄存器的常量（与 include/anlogic_pci_ioctl.h 里的偏移一起用） */
#define RKA1_CANCEL_WAIT_MS      50u    /* CANCEL 后等 busy/draining 清零的上限 */
#define RKA1_CANCEL_POLL_US      200u   /* 轮询间隔 */
#define RKA1_ACK_POLL_MS         20u    /* ACK 后等 held 释放的上限 */

static int rka1_read(struct anlogic_pci_hw *hw, u32 off, u32 *val)
{
    return anlogic_pci_link_read32(hw, off, val);
}

static int rka1_write(struct anlogic_pci_hw *hw, u32 off, u32 val)
{
    return anlogic_pci_link_write32(hw, off, val);
}

/*
 * 探测安全扩展：EXT_MAGIC/EXT_VERSION 必须都对。失败一律 fail closed。
 * 本函数只读，不改任何状态，可以在任何一次 open 里安全调用。
 */
int anlogic_pci_rka1_probe(struct anlogic_pci_hw *hw)
{
    u32 magic = 0, ver = 0;
    int rv;

    if (!hw || !hw->pdev)
        return -EINVAL;

    rv = rka1_read(hw, ANLOGIC_BAR_EXT_MAGIC_OFF, &magic);
    if (rv < 0)
        return rv;
    if (magic != ANLOGIC_RKA1_EXT_MAGIC) {
        pr_err("RKA1：0x118 EXT_MAGIC=0x%08x != 0x%08x（旧映像或被复用寄存器）。"
               "本驱动不做旧行为回退，推帧路径关闭。\n",
               magic, ANLOGIC_RKA1_EXT_MAGIC);
        hw->rka1_ext_ok = 0;
        return -EOPNOTSUPP;
    }
    rv = rka1_read(hw, ANLOGIC_BAR_EXT_VERSION_OFF, &ver);
    if (rv < 0)
        return rv;
    if (ANLOGIC_RKA1_EXT_VERSION_MAJOR(ver) !=
        ANLOGIC_RKA1_EXT_VERSION_MAJOR(ANLOGIC_RKA1_EXT_VERSION)) {
        pr_err("RKA1：0x11C EXT_VERSION=0x%08x 主版本不符（要 major %u）。\n",
               ver, ANLOGIC_RKA1_EXT_VERSION_MAJOR(ANLOGIC_RKA1_EXT_VERSION));
        hw->rka1_ext_ok = 0;
        return -EOPNOTSUPP;
    }
    hw->rka1_ext_ok = 1;
    hw->rka1_ext_version = ver;
    return 0;
}

/* 读 TX_STATUS（0x114）。任何 RKA1 操作前后都用它复核状态 */
int anlogic_pci_rka1_status(struct anlogic_pci_hw *hw, u32 *status)
{
    int rv = rka1_read(hw, ANLOGIC_BAR_TX_STATUS_OFF, status);
    if (rv == 0)
        hw->rka1_tx_status = *status;
    return rv;
}

/*
 * 配置授权：CAP_BYTES / SESSION_ID 都是「暂存」，ARM 时由 FPGA 一起锁存并校验。
 * 要求：
 *   · cap_bytes >= 本帧 dma_bytes（FPGA 侧逐帧再核一次，两边都要挡）；
 *   · session 非零且与上次不同（每次重连/卸载恢复换新会话，避免旧门铃/旧 ACK 被认）。
 * 注意：busy/held/draining 期间写这两个寄存器会被 FPGA 拒绝（置 0xC8=10），
 * 所以本函数先看 TX_STATUS，有活动就直接返回 -EBUSY，不去踩那个拒绝路径。
 */
int anlogic_pci_rka1_configure(struct anlogic_pci_hw *hw, u32 cap_bytes, u32 session)
{
    u32 st = 0;
    int rv;

    if (!hw || !hw->pdev)
        return -EINVAL;
    if (!hw->rka1_ext_ok)
        return -EOPNOTSUPP;
    if (!session)
        return -EINVAL;
    if (cap_bytes < ANLOGIC_FRAME_DMA_BYTES) {
        pr_err("RKA1：授权容量 %u 小于一帧 dma_bytes %u，拒绝配置。\n",
               cap_bytes, ANLOGIC_FRAME_DMA_BYTES);
        return -ERANGE;
    }
    if (cap_bytes & 0xfffu) {
        pr_err("RKA1：授权容量 %u 不是 4KiB 整数倍，拒绝配置。\n", cap_bytes);
        return -EINVAL;
    }

    rv = anlogic_pci_rka1_status(hw, &st);
    if (rv < 0)
        return rv;
    if (st & (ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY |
              ANLOGIC_RKA1_ST_HELD   | ANLOGIC_RKA1_ST_DRAINING)) {
        pr_err("RKA1：活动期间（TX_STATUS=0x%08x）拒绝改授权配置。\n", st);
        return -EBUSY;
    }

    rv = rka1_write(hw, ANLOGIC_BAR_TX_CAP_OFF, cap_bytes);
    if (rv < 0)
        return rv;
    rv = rka1_write(hw, ANLOGIC_BAR_TX_SESSION_OFF, session);
    if (rv < 0)
        return rv;

    /* 读回复核：写没生效（老映像）必须当场看出来 */
    {
        u32 rb_cap = 0, rb_sess = 0;
        rv = rka1_read(hw, ANLOGIC_BAR_TX_CAP_OFF, &rb_cap);
        if (rv < 0)
            return rv;
        rv = rka1_read(hw, ANLOGIC_BAR_TX_SESSION_OFF, &rb_sess);
        if (rv < 0)
            return rv;
        if (rb_cap != cap_bytes || rb_sess != session) {
            pr_err("RKA1：授权配置回读不符（cap 0x%08x/0x%08x, sess 0x%08x/0x%08x）。\n",
                   rb_cap, cap_bytes, rb_sess, session);
            return -EIO;
        }
    }
    hw->rka1_cap_bytes = cap_bytes;
    hw->rka1_session = session;
    return 0;
}

/*
 * 一键 ARM_ONE_SHOT：写 0x108 = 0x1（W1，读恒 0）。
 *   · 写前：TX_STATUS 必须没有 armed/busy/held/draining（否则 FPGA 也会拒，但先在
 *     驱动侧挡掉，错误信息更清楚）；
 *   · 写后立刻读 TX_STATUS：armed 必须置起；若 rejected 置起说明 FPGA 校验拒绝
 *    （容量/session/对齐/链路/MPS 等），返回 -EINVAL 并把 0xC8 打出来。
 * 返回 0 只表示「额度已给」；是否真的推完要看 held 或 0xC0/0xC8。
 */
int anlogic_pci_rka1_arm_one_shot(struct anlogic_pci_hw *hw)
{
    u32 st = 0, err = 0;
    int rv;

    if (!hw || !hw->pdev)
        return -EINVAL;
    if (!hw->rka1_ext_ok)
        return -EOPNOTSUPP;
    if (!hw->rka1_session || !hw->rka1_cap_bytes) {
        pr_err("RKA1：还没配置授权（cap/session），拒绝 ARM。\n");
        return -EINVAL;
    }

    rv = anlogic_pci_rka1_status(hw, &st);
    if (rv < 0)
        return rv;
    if (st & (ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY |
              ANLOGIC_RKA1_ST_HELD   | ANLOGIC_RKA1_ST_DRAINING)) {
        pr_err("RKA1：TX_STATUS=0x%08x 不是空闲（armed/busy/held/draining），拒绝 ARM。\n",
               st);
        return -EBUSY;
    }

    rv = rka1_write(hw, ANLOGIC_BAR_TX_CMD_OFF, ANLOGIC_RKA1_CMD_ARM);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_rka1_status(hw, &st);
    if (rv < 0)
        return rv;
    if (st & ANLOGIC_RKA1_ST_REJECT) {
        rka1_read(hw, ANLOGIC_BAR_PUSH_ERR_OFF, &err);
        pr_err("RKA1：ARM 被 FPGA 拒绝（TX_STATUS.bit3=1，0xC8=%u）。"
               "检查 cap/session/dst/db 对齐与链路/MPS 状态。\n", err);
        return -EINVAL;
    }
    if (!(st & ANLOGIC_RKA1_ST_ARMED)) {
        pr_err("RKA1：写完 0x108 后 TX_STATUS=0x%08x 没有 armed，写入没生效。\n", st);
        return -EIO;
    }
    return 0;
}

/*
 * ACK_RELEASE：ACK_SEQ/ACK_SESSION 必须与 FPGA held 的 (session,seq) 逐位相等。
 * 驱动只做参数与状态搬运，匹配判定在 RTL 里做（错误 ACK 不会覆盖 held 槽）。
 * 释放成功 = TX_STATUS.held 变 0。
 */
int anlogic_pci_rka1_ack_release(struct anlogic_pci_hw *hw, u32 seq, u32 session)
{
    u32 st = 0, err = 0;
    unsigned int waited = 0;
    int rv;

    if (!hw || !hw->pdev)
        return -EINVAL;
    if (!hw->rka1_ext_ok)
        return -EOPNOTSUPP;
    if (!seq || !session)
        return -EINVAL;

    rv = anlogic_pci_rka1_status(hw, &st);
    if (rv < 0)
        return rv;
    if (!(st & ANLOGIC_RKA1_ST_HELD)) {
        pr_err("RKA1：ACK 时 FPGA 不在 held（TX_STATUS=0x%08x），拒绝。\n", st);
        return -EINVAL;
    }

    rv = rka1_write(hw, ANLOGIC_BAR_TX_ACK_SESS_OFF, session);
    if (!rv)
        rv = rka1_write(hw, ANLOGIC_BAR_TX_ACK_SEQ_OFF, seq);
    if (!rv)
        rv = rka1_write(hw, ANLOGIC_BAR_TX_CMD_OFF, ANLOGIC_RKA1_CMD_ACK);
    if (rv < 0)
        return rv;

    while (waited < RKA1_ACK_POLL_MS) {
        rv = anlogic_pci_rka1_status(hw, &st);
        if (rv < 0)
            return rv;
        if (!(st & ANLOGIC_RKA1_ST_HELD))
            return 0;
        usleep_range(1000, 2000);
        waited++;
    }
    rka1_read(hw, ANLOGIC_BAR_PUSH_ERR_OFF, &err);
    pr_err("RKA1：ACK 后 %u ms 仍 held（TX_STATUS=0x%08x，0xC8=%u）——"
           "seq/session 与 held 值不匹配？\n", waited, st, err);
    return -EAGAIN;
}

/*
 * CANCEL：ARMED 直接放弃；TX 中把**已开始的包补完**（绝不截半包）、不发成功门铃、
 * 排空后回 IDLE；HELD 则拆掉 held 槽。等 busy/draining 清零再返回 —— 只有清零后，
 * 上层才允许释放收帧内存/复用那块缓冲。
 */
int anlogic_pci_rka1_cancel(struct anlogic_pci_hw *hw)
{
    u32 st = 0;
    unsigned int waited = 0;
    int rv;

    if (!hw || !hw->pdev)
        return -EINVAL;
    if (!hw->rka1_ext_ok)
        return -EOPNOTSUPP;

    rv = rka1_write(hw, ANLOGIC_BAR_TX_CMD_OFF, ANLOGIC_RKA1_CMD_CANCEL);
    if (rv < 0)
        return rv;

    while (waited < RKA1_CANCEL_WAIT_MS) {
        rv = anlogic_pci_rka1_status(hw, &st);
        if (rv < 0)
            return rv;
        if (!(st & (ANLOGIC_RKA1_ST_BUSY | ANLOGIC_RKA1_ST_DRAINING |
                    ANLOGIC_RKA1_ST_HELD | ANLOGIC_RKA1_ST_ARMED)))
            return 0;
        usleep_range(RKA1_CANCEL_POLL_US, RKA1_CANCEL_POLL_US * 2);
        waited++;
    }
    pr_err("RKA1：CANCEL 后 %u ms 仍有活动（TX_STATUS=0x%08x）。\n", waited, st);
    return -EAGAIN;
}

/* ioctl 入口：ANLOGIC_IOCRKA1 */
long anlogic_pci_rka1_ioctl(struct anlogic_pci_hw *hw, unsigned long arg)
{
    struct anlogic_ioc_rka1 rq;
    int rv;

    if (!hw)
        return -EINVAL;
    if (copy_from_user(&rq, (void __user *)arg, sizeof(rq)))
        return -EFAULT;

    switch (rq.op) {
    case ANLOGIC_RKA1_OP_PROBE:
        rv = anlogic_pci_rka1_probe(hw);
        break;
    case ANLOGIC_RKA1_OP_CONFIGURE:
        rv = anlogic_pci_rka1_configure(hw, rq.cap_bytes, rq.session);
        break;
    case ANLOGIC_RKA1_OP_ARM:
        rv = anlogic_pci_rka1_arm_one_shot(hw);
        break;
    case ANLOGIC_RKA1_OP_ACK:
        rv = anlogic_pci_rka1_ack_release(hw, rq.ack_seq, rq.ack_session);
        break;
    case ANLOGIC_RKA1_OP_CANCEL:
        rv = anlogic_pci_rka1_cancel(hw);
        break;
    default:
        return -EINVAL;
    }

    /* 只要寄存器读得动，就把当前状态填回去（失败时也尽量给诊断值） */
    rka1_read(hw, ANLOGIC_BAR_EXT_MAGIC_OFF, &rq.ext_magic);
    rka1_read(hw, ANLOGIC_BAR_EXT_VERSION_OFF, &rq.ext_version);
    anlogic_pci_rka1_status(hw, &rq.tx_status);
    rka1_read(hw, ANLOGIC_BAR_PUSH_ERR_OFF, &rq.push_err);
    rka1_read(hw, ANLOGIC_BAR_PUSH_STATUS_OFF, &rq.push_status);
    rka1_read(hw, ANLOGIC_BAR_PUSH_DONE_OFF, &rq.push_done);
    rq.armed    = !!(rq.tx_status & ANLOGIC_RKA1_ST_ARMED);
    rq.busy     = !!(rq.tx_status & ANLOGIC_RKA1_ST_BUSY);
    rq.held     = !!(rq.tx_status & ANLOGIC_RKA1_ST_HELD);
    rq.rejected = !!(rq.tx_status & ANLOGIC_RKA1_ST_REJECT);

    if (copy_to_user((void __user *)arg, &rq, sizeof(rq)))
        return -EFAULT;

    return rv;
}
