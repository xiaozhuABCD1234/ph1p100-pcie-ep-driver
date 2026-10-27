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
#define pr_fmt(fmt) KBUILD_MODNAME ": [rka1] " fmt

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>

#include "anlogic_pci.h"

/* RTL 里私有寄存器的常量（与 include/anlogic_pci_ioctl.h 里的偏移一起用） */
#define RKA1_CANCEL_WAIT_MS      50u    /* CANCEL 后等 busy/draining 清零的上限 */
#define RKA1_CANCEL_POLL_US      200u   /* 轮询间隔 */
#define RKA1_ACK_WAIT_MS         20u    /* ACK 后等 held 释放的上限（每轮睡 1~2ms） */
#define RKA1_QUIESCE_WAIT_MS     60u    /* ARM 前把 armed/held/busy 收回 IDLE 的上限 */

static int rka1_read(struct anlogic_pci_hw *hw, u32 off, u32 *val)
{
    return anlogic_pci_link_read32(hw, off, val);
}

static int rka1_write(struct anlogic_pci_hw *hw, u32 off, u32 val)
{
    return anlogic_pci_link_write32(hw, off, val);
}

/* 把 armed/busy/held/draining 收回 IDLE；定义在 configure 之后 */
static int rka1_quiesce_to_idle(struct anlogic_pci_hw *hw, u32 st);

/*
 * 复核 0xB0/0xB4 与 0xB8/0xBC 两对寄存器是否仍是驱动写下去的槽 0/门铃。
 * link_armed 是软件状态，FPGA 被重新配置后寄存器可能已归零而软件还记着 1；
 * 每次 ARM 前读回来比对一次，避免把帧打到 0 或旧地址。
 */
static int rka1_push_regs_match(struct anlogic_pci_hw *hw)
{
    u32 dst_lo = 0, dst_hi = 0, db_lo = 0, db_hi = 0;
    u64 dst, db;
    int rv;

    rv = rka1_read(hw, ANLOGIC_BAR_PUSH_DST_LO_OFF, &dst_lo);
    if (!rv)
        rv = rka1_read(hw, ANLOGIC_BAR_PUSH_DST_HI_OFF, &dst_hi);
    if (!rv)
        rv = rka1_read(hw, ANLOGIC_BAR_PUSH_DB_LO_OFF, &db_lo);
    if (!rv)
        rv = rka1_read(hw, ANLOGIC_BAR_PUSH_DB_HI_OFF, &db_hi);
    if (rv < 0)
        return rv;

    dst = ((u64)dst_hi << 32) | dst_lo;
    db = ((u64)db_hi << 32) | db_lo;
    if (dst != hw->link_push_dst || db != hw->link_push_db) {
        pr_err("0xB0/0xB4=0x%llx、0xB8/0xBC=0x%llx 与驱动记录 0x%llx/0x%llx 不一致。\n",
               (unsigned long long)dst, (unsigned long long)db,
               (unsigned long long)hw->link_push_dst,
               (unsigned long long)hw->link_push_db);
        return -EAGAIN;
    }

    return 0;
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
        pr_err("0x118 EXT_MAGIC=0x%08x != 0x%08x（旧映像或被复用寄存器）。"
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
        pr_err("0x11C EXT_VERSION=0x%08x 主版本不符（要 major %u）。\n",
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
 * 注意：busy/held/draining 期间写这两个寄存器会被 FPGA 拒绝（置 0xC8=10）。
 * 所以本函数先看 TX_STATUS：若还有活动，先用 ACK/CANCEL 收回 IDLE 再写。
 * 这样用户态在探测留下的 held 帧之后仍能换新 session 重新授权；收回时
 * 优先 ACK（不产生 E_CANCEL），ACK 不匹配才退回 CANCEL。
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
    /*
     * 容量下限用当前收帧缓冲的实际槽长，而不是 1080p 的固定常量：
     * 4K R10F 槽（10371072）和 640x480 槽（385024）都必须能通过，同时
     * 不能授权超过已分配缓冲的容量。
     */
    {
        u32 min_bytes = hw->frame_slot_bytes ? hw->frame_slot_bytes :
                                                ANLOGIC_FRAME_DMA_BYTES;

        if (cap_bytes < min_bytes) {
            pr_err("授权容量 %u 小于一帧传输长度 %u，拒绝配置。\n",
                   cap_bytes, min_bytes);
            return -ERANGE;
        }
        if (hw->frame_size && (u64)cap_bytes > (u64)hw->frame_size) {
            pr_err("授权容量 %u 超过收帧缓冲 %zu，拒绝配置。\n",
                   cap_bytes, hw->frame_size);
            return -ERANGE;
        }
    }
    if (cap_bytes & 0xfffu) {
        pr_err("授权容量 %u 不是 4KiB 整数倍，拒绝配置。\n", cap_bytes);
        return -EINVAL;
    }

    rv = anlogic_pci_rka1_status(hw, &st);
    if (rv < 0)
        return rv;
    if (st & (ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY |
              ANLOGIC_RKA1_ST_HELD   | ANLOGIC_RKA1_ST_DRAINING)) {
        pr_debug("配置授权前 TX_STATUS=0x%08x 非空闲，先 ACK/CANCEL 收回 IDLE。\n",
                 st);
        rv = rka1_quiesce_to_idle(hw, st);
        if (rv < 0) {
            pr_err("收回 IDLE 失败 %d，拒绝改授权配置。\n", rv);
            return rv;
        }
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
            pr_err("授权配置回读不符（cap 0x%08x/0x%08x, sess 0x%08x/0x%08x）。\n",
                   rb_cap, cap_bytes, rb_sess, session);
            return -EIO;
        }
    }
    hw->rka1_cap_bytes = cap_bytes;
    hw->rka1_session = session;
    return 0;
}

/*
 * 把 TX 引擎收回 IDLE（ARM 幂等用）。
 *   HELD       → 先用 (0xC0, 驱动会话) 做一次 ACK：RTL 的 held_seq 就是门铃值
 *                （claim 时 db_val = st_done+1，推完 st_done 自增），0xC0 逐位匹配；
 *                比直接 CANCEL 温和（不产生 E_CANCEL、不置 bit3）。
 *   ARMED      → CANCEL：额度还没开始任何传输，丢弃零代价。
 *   BUSY/DRAIN → CANCEL：RTL 会把已开始的包补完再排空，绝不截半包。
 * 返回 0 表示 armed/busy/held/draining 已全清。
 */
static int rka1_quiesce_to_idle(struct anlogic_pci_hw *hw, u32 st)
{
    const u32 busy_mask = ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY |
                          ANLOGIC_RKA1_ST_HELD  | ANLOGIC_RKA1_ST_DRAINING;
    ktime_t start = ktime_get();
    s64 elapsed_ms;
    u32 done = 0;
    int rv;

    /*
     * 用「读状态 → 按状态动作 → 再读」的轮询，而不是按调用方传进来的 st 一次性决策：
     * 板上实测出现过 st 读到 HELD、几微秒后 ACK 内部再读已经是 ARMED 的竞态，
     * 一次性决策会走错分支（结果 E_CANCEL=7 而 held 没释放，ARM 一直被拒）。
     */
    while (ktime_ms_delta(ktime_get(), start) < RKA1_QUIESCE_WAIT_MS) {
        if (!(st & busy_mask))
            return 0;
        if (st & ANLOGIC_RKA1_ST_HELD) {
            rv = rka1_read(hw, ANLOGIC_BAR_PUSH_DONE_OFF, &done);
            if (!rv && done) {
                rv = anlogic_pci_rka1_ack_release(hw, done,
                                                  hw->rka1_session);
                if (rv < 0) {
                    /*
                     * held 槽里的 (session,seq) 可能是上一轮遗留的，用当前
                     * 会话 ACK 不掉；此时不能死等，CANCEL 是更强制的收回。
                     */
                    pr_warn("held 的 ACK 未确认（%d），改用 CANCEL 收回。\n",
                            rv);
                    (void)anlogic_pci_rka1_cancel(hw);
                }
            } else {
                (void)anlogic_pci_rka1_cancel(hw);
            }
        } else {
            (void)anlogic_pci_rka1_cancel(hw);
        }
        usleep_range(2000, 3000);
        rv = anlogic_pci_rka1_status(hw, &st);
        if (rv < 0)
            return rv;
    }
    if (st & busy_mask) {
        elapsed_ms = ktime_ms_delta(ktime_get(), start);
        pr_err("收回 IDLE 超时（%lld ms，TX_STATUS=0x%08x）。\n",
               elapsed_ms, st);
        return -EBUSY;
    }
    return 0;
}

/*
 * 一键 ARM_ONE_SHOT：写 0x108 = 0x1（W1，读恒 0）。
 *   · 写前：TX_STATUS 必须没有 armed/busy/held/draining（否则先自愈回 IDLE，
 *     自愈失败才拒绝，错误信息更清楚）；
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
        pr_err("还没配置授权（cap/session），拒绝 ARM。\n");
        return -EINVAL;
    }
    /*
     * 没有完整链路就绪状态时绝不允许 ARM：用户态可以直接发 OP_ARM，
     * 若 link_arm 在写完授权配置后、写推帧地址前失败，0xB0/0xB8 可能还是
     * 0 或上一次的旧值。这里把“地址已写、使能位已置、目标在缓冲内”作为硬门槛。
     */
    if (!hw->link_armed) {
        pr_err("链路/推帧尚未初始化（link_armed=0），拒绝 ARM。\n");
        return -EAGAIN;
    }
    if (!hw->frame_virt || !hw->frame_slot_bytes ||
        hw->link_push_dst != (u64)hw->frame_phys + hw->frame_hdr_bytes ||
        hw->link_push_db != (u64)hw->frame_phys + hw->frame_mb_off +
                            offsetof(struct anlogic_frame_mailbox, push_dbell)) {
        pr_err("推帧目标/门铃地址不是驱动配置的槽 0（dst=0x%llx db=0x%llx），拒绝 ARM。\n",
               (unsigned long long)hw->link_push_dst,
               (unsigned long long)hw->link_push_db);
        return -EINVAL;
    }
    if (hw->frame_size &&
        ((hw->link_push_dst < hw->frame_phys) ||
         (hw->link_push_dst - hw->frame_phys + hw->frame_slot_bytes >
          hw->frame_size) ||
         (hw->link_push_db < hw->frame_phys) ||
         (hw->link_push_db - hw->frame_phys + 4 > hw->frame_size))) {
        pr_err("推帧目标/门铃越出收帧缓冲，拒绝 ARM。\n");
        return -EINVAL;
    }
    rv = rka1_push_regs_match(hw);
    if (rv < 0) {
        if (rv == -EAGAIN) {
            pr_err("FPGA 里的推帧地址已被改写（可能重新配置过），拒绝 ARM。\n");
            hw->link_armed = 0;
        }
        return rv;
    }

    rv = anlogic_pci_rka1_status(hw, &st);
    if (rv < 0)
        return rv;
    if (st & (ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY |
              ANLOGIC_RKA1_ST_HELD   | ANLOGIC_RKA1_ST_DRAINING)) {
        /*
         * 2026-10-28：原来这里直接 -EBUSY 硬拒，于是「上一次没 ACK / 没 CANCEL」
         * 会让之后所有抓帧都失败（板上实测：第二次抓帧 0xC0 不涨、落盘还是旧缓冲）。
         * 改成先自愈回 IDLE、重写授权，再照常 ARM。
         */
        pr_debug("ARM 前 TX_STATUS=0x%08x 不是空闲，先收回 IDLE（幂等）。\n", st);
        rv = rka1_quiesce_to_idle(hw, st);
        if (rv < 0) {
            pr_err("收回 IDLE 失败 %d，拒绝 ARM。\n", rv);
            return rv;
        }
        /* 活动期的 CONFIGURE 可能被 RTL 以 cfg_reject 拒过（0xC8=10），
         * 暂存寄存器里还是旧值，这里重写一遍并回读复核再 ARM。
         */
        rv = anlogic_pci_rka1_configure(hw, hw->rka1_cap_bytes,
                                        hw->rka1_session);
        if (rv < 0)
            return rv;
        rv = anlogic_pci_rka1_status(hw, &st);
        if (rv < 0)
            return rv;
        if (st & (ANLOGIC_RKA1_ST_ARMED | ANLOGIC_RKA1_ST_BUSY |
                  ANLOGIC_RKA1_ST_HELD   | ANLOGIC_RKA1_ST_DRAINING)) {
            pr_err("收回 IDLE 后 TX_STATUS=0x%08x 仍不空闲，拒绝 ARM。\n", st);
            return -EBUSY;
        }
    }

    rv = rka1_write(hw, ANLOGIC_BAR_TX_CMD_OFF, ANLOGIC_RKA1_CMD_ARM);
    if (rv < 0)
        return rv;
    rv = anlogic_pci_rka1_status(hw, &st);
    if (rv < 0)
        return rv;
    if (st & ANLOGIC_RKA1_ST_REJECT) {
        rka1_read(hw, ANLOGIC_BAR_PUSH_ERR_OFF, &err);
        /*
         * E_ARM_REJ 最常见的原因不是配置错，而是「相机还没产出第一帧」：
         * RTL 的 arm_reject 里有 (ddr_status[0]==0) 一条，DDR 里没有可用
         * 源帧时必然拒一次。上电流程里 CTRL 刚使能、传感器还在出第一帧，
         * 这一次拒绝是正常的，调用方（anlogic_pci_link_arm）会等帧重试。
         * 这里降到 pr_debug 免得重试期间刷屏；配置真错的码仍按 err 打印。
         */
        if (err == ANLOGIC_PUSH_ERR_ARM_REJ)
            pr_debug("ARM 被拒（0xC8=%u E_ARM_REJ，通常是相机还没出第一帧）。", err);
        else
            pr_err("ARM 被 FPGA 拒绝（TX_STATUS.bit3=1，0xC8=%u）。"
                   "检查 cap/session/dst/db 对齐与链路/MPS 状态。", err);
        return -EINVAL;
    }
    if (!(st & ANLOGIC_RKA1_ST_ARMED)) {
        pr_err("写完 0x108 后 TX_STATUS=0x%08x 没有 armed，写入没生效。\n", st);
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
    ktime_t start;
    s64 elapsed_ms;
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
        /*
         * 幂等：本来就没有 held 槽可释放。若还停在 ARMED，说明这个额度一帧都没推
         * （源没再发布 / claim 被拒），顺手 CANCEL 收回，别留给下一次 ARM；
         * 其余情况直接成功返回，让上层用 0xC0 增量判「这次到底推没推」。
         */
        if (st & ANLOGIC_RKA1_ST_ARMED) {
            pr_debug("ACK 时还在 ARMED（该额度未推任何帧），改为 CANCEL 收回。\n");
            return anlogic_pci_rka1_cancel(hw);
        }
        pr_debug("ACK 时不在 held（TX_STATUS=0x%08x），视为已释放。\n", st);
        return 0;
    }

    rv = rka1_write(hw, ANLOGIC_BAR_TX_ACK_SESS_OFF, session);
    if (!rv)
        rv = rka1_write(hw, ANLOGIC_BAR_TX_ACK_SEQ_OFF, seq);
    if (!rv)
        rv = rka1_write(hw, ANLOGIC_BAR_TX_CMD_OFF, ANLOGIC_RKA1_CMD_ACK);
    if (rv < 0)
        return rv;

    start = ktime_get();
    while (ktime_ms_delta(ktime_get(), start) < RKA1_ACK_WAIT_MS) {
        rv = anlogic_pci_rka1_status(hw, &st);
        if (rv < 0)
            return rv;
        if (!(st & ANLOGIC_RKA1_ST_HELD))
            return 0;
        usleep_range(1000, 2000);
    }
    elapsed_ms = ktime_ms_delta(ktime_get(), start);
    rka1_read(hw, ANLOGIC_BAR_PUSH_ERR_OFF, &err);
    pr_err("ACK 后 %lld ms 仍 held（TX_STATUS=0x%08x，0xC8=%u）——"
           "seq/session 与 held 值不匹配？\n", elapsed_ms, st, err);
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
    ktime_t start;
    s64 elapsed_ms;
    int rv;

    if (!hw || !hw->pdev)
        return -EINVAL;
    if (!hw->rka1_ext_ok)
        return -EOPNOTSUPP;

    rv = rka1_write(hw, ANLOGIC_BAR_TX_CMD_OFF, ANLOGIC_RKA1_CMD_CANCEL);
    if (rv < 0)
        return rv;

    start = ktime_get();
    while (ktime_ms_delta(ktime_get(), start) < RKA1_CANCEL_WAIT_MS) {
        rv = anlogic_pci_rka1_status(hw, &st);
        if (rv < 0)
            return rv;
        if (!(st & (ANLOGIC_RKA1_ST_BUSY | ANLOGIC_RKA1_ST_DRAINING |
                    ANLOGIC_RKA1_ST_HELD | ANLOGIC_RKA1_ST_ARMED)))
            return 0;
        usleep_range(RKA1_CANCEL_POLL_US, RKA1_CANCEL_POLL_US * 2);
    }
    elapsed_ms = ktime_ms_delta(ktime_get(), start);
    pr_err("CANCEL 后 %lld ms 仍有活动（TX_STATUS=0x%08x）。\n",
           elapsed_ms, st);
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
        /*
         * ack_session == 0 = 「用驱动在 probe 里配置的那个会话」。会话号只存在于
         * 内核（anlogic_pci_link_arm 里用 jiffies 生成），用户态无从得知；不给这条
         * 退路的话，probe 时那次 ARM 推来的帧就永远 ACK 不掉，held 会卡住后续 ARM。
         * 安全边界不变：RTL 仍按 held 槽里锁存的 (session, seq) 逐位比对，
         * 传 0 只是让驱动补上自己那份，不会放宽匹配。
         */
        if (!rq.ack_session)
            rq.ack_session = hw->rka1_session;
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
