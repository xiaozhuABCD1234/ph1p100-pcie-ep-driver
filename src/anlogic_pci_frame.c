// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动：收帧缓冲层。
 *
 * 目标形态：FPGA 侧用 MWr 把整帧推进 RK3588 的一块 DMA 缓冲，用户态取帧显示。
 * 本层只做「缓冲 + 接口」，真正的收帧等 FPGA 侧打通后再验：
 *
 *   1) 分配：dma_alloc_coherent 一块物理连续、页对齐的一致性内存，大小由
 *      module param 决定（默认 1 槽 × 640x480 灰度 = 307200 字节/槽）。
 *      每槽步长页对齐，推帧目标按 phys_addr + stride * n + hdr_bytes 算（= 像素区起点）。
 *   2) 接口：ANLOGIC_IOCFRAMEINFO 把物理地址、大小、几何与序号/计数报给用户态；
 *      ANLOGIC_IOCFRAMEWAIT 提供「帧就绪」轮询（序号 + 计数）—— 本 EP 目前只有
 *      Legacy 中断、没有 MSI，所以不做中断，只轮询邮箱。
 *   3) 推帧门铃：FPGA 推帧引擎只往门铃字写「已推帧数」，不写槽头、不碰
 *      mailbox.seq/count（见 include/anlogic_pci_ioctl.h 的说明）。所以本层在
 *      轮询时盯的是门铃字，见到它变大就把对应槽的槽头与 mailbox.seq/count 补齐
 *      —— 用户态（frame_grab）照旧只读 mailbox.seq，协议面不变。
 *   4) 释放：模块卸载（hw_close）与 probe 出错路径都经 anlogic_pci_frame_free()，
 *      保证 dma_free_coherent 一定被调用。
 *
 * 内存布局与握手约定写在 include/anlogic_pci_ioctl.h（驱动/用户态/FPGA 三方共用）。
 *
 * 与 BAR 访问校验的关系：本层**完全不碰 BAR 寄存器**（不做任何 MMIO），只读写自己
 * 分配的普通内存，所以 BAR 访问的三条校验（偏移白名单、先验 MAGIC、
 * 单笔不超过 4 个双字）在这里没有可违反的地方。对 mailbox 的读用 READ_ONCE +
 * dma_rmb()：先看到 FPGA 写的序号，再去看它写的帧数据。
 * 帧缓冲节点的 mmap 映射的是系统内存（不是 MMIO），但仍按既有的 mmap
 * 溢出安全判据做边界夹紧，并且只允许只读映射（不把 FPGA 的 MWr 目标交给用户态乱写）。
 */
#include <linux/build_bug.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "anlogic_pci.h"

/* 一帧的默认几何：640x480 灰度 = 307200 字节 */
#define ANLOGIC_FRAME_DEF_WIDTH     640u
#define ANLOGIC_FRAME_DEF_HEIGHT    480u
/*
 * 默认单槽。推帧引擎只往主机给的那一个目标地址写（0xB0/0xB4 由驱动探测时写好、
 * 之后不逐帧轮换），所以帧永远落在同一个槽里；多槽下用户态按 (seq-1)%slots 取槽
 * 会取到没数据的槽。要跑别的路径（PIO 0x60 / 只读 DDR3）时可以用 frame_slots 开多槽。
 */
#define ANLOGIC_FRAME_DEF_SLOTS     1u
/* 轮询间隔，也是「等帧」时的睡眠粒度 */
#define ANLOGIC_FRAME_POLL_MS       10u
/* 一帧像素字节数的合理上限，防止 module param 写出荒唐的分配量 */
#define ANLOGIC_FRAME_MAX_W         65535u
#define ANLOGIC_FRAME_MAX_H         65535u

static unsigned int frame_width = ANLOGIC_FRAME_DEF_WIDTH;
module_param(frame_width, uint, 0444);
MODULE_PARM_DESC(frame_width, "收帧缓冲的每帧宽度（像素），默认 640");

static unsigned int frame_height = ANLOGIC_FRAME_DEF_HEIGHT;
module_param(frame_height, uint, 0444);
MODULE_PARM_DESC(frame_height, "收帧缓冲的每帧高度（像素），默认 480");

/* 0 表示用 frame_width * frame_height（默认 640x480 灰度 = 307200 字节） */
static unsigned int frame_slot_bytes;
module_param(frame_slot_bytes, uint, 0444);
MODULE_PARM_DESC(frame_slot_bytes, "一帧像素字节数，0 = frame_width*frame_height（默认 307200）");

static unsigned int frame_slots = ANLOGIC_FRAME_DEF_SLOTS;
module_param(frame_slots, uint, 0444);
MODULE_PARM_DESC(frame_slots,
                 "收帧缓冲的槽位数（1..16），默认 1（推帧目标固定，多槽只有槽 0 有推来的帧）");

/*
 * mailbox 与槽头的字段偏移是 FPGA / 驱动 / 用户态三方共用的 ABI，偏移算错的话
 * FPGA 的 MWr 就打偏了，所以这里在编译期钉死（用户态 frame_grab.c 里有同样的检查）。
 */
static_assert(sizeof(struct anlogic_frame_mailbox) == 64,
              "mailbox 必须是 64 字节，否则 FPGA 的偏移约定对不上");
static_assert(sizeof(struct anlogic_frame_slot_hdr) == 16,
              "槽头必须是 16 字节，否则像素起始偏移对不上");

/*
 * 读 mailbox 的序号与计数。
 * FPGA 先写完整帧（含槽头）再写 mailbox.seq，所以 CPU 侧要保证「序号读了之后才去
 * 读帧数据」不被编译器/CPU 提前 —— dma_rmb() 放在读 seq 之后（生产者-消费者序）。
 *
 * 可见性：本板 PCIe RC 的设备树节点没有标 dma-coherent，kernel/dma/direct.c 的
 * dma_pgprot() 会把这块一致性内存映射成 Normal-NC（内核别名与用户态 mmap 都是），
 * FPGA 的 MWr 落到 DRAM 之后 CPU/用户态直接读就能看到，不需要额外 invalidate。
 * 若以后把 RC 标成 dma-coherent（改按 cacheable 映射），这里读序号之前必须补
 * dma_sync_single_for_cpu()，否则会读到脏 cache line。
 */
static void anlogic_pci_frame_seq_read(struct anlogic_pci_hw *hw, u32 *seq,
                                       u32 *count)
{
    *seq = READ_ONCE(hw->frame_mb->seq);
    dma_rmb();
    *count = READ_ONCE(hw->frame_mb->count);
}

/*
 * 推帧门铃 → mailbox 协议的补齐（见 include/anlogic_pci_ioctl.h 里「本板推帧通路的
 * 实际分工」那段）：门铃字装的是 FPGA 侧的「已推帧数」（1 起、每帧 +1）；驱动见到它
 * 比上次大了，就
 *   ① 把新帧的槽头（seq/len/flags）写进该帧所在的槽，
 *   ② 跟上 mailbox.count，
 *   ③ **最后**才更新 mailbox.seq —— 与头文件里给 FPGA 定的是同一个顺序：
 *      用户态看到序号变化时，槽头一定已经在内存里了（序号是最后一笔写）。
 * 推帧目标地址由驱动在探测时写好且不逐帧轮换，所以推来的帧都在同一个槽里；
 * frame_slots 默认 1，(done-1)%slots 恒为 0。
 * 读门铃用 READ_ONCE + dma_rmb()：先看到门铃，再去看它标记的那一帧。
 */
static void anlogic_pci_frame_push_sync(struct anlogic_pci_hw *hw)
{
    struct anlogic_frame_mailbox *mb = hw->frame_mb;
    struct anlogic_frame_slot_hdr *hdr;
    unsigned int slot;
    u32 done;

    if (!mb || !hw->frame_slots || !hw->frame_stride)
        return;

    done = READ_ONCE(mb->push_dbell);
    dma_rmb();
    if (!done || done == hw->frame_push_done)
        return;

    slot = (done - 1) % hw->frame_slots;
    hdr = (struct anlogic_frame_slot_hdr *)((u8 *)hw->frame_virt +
                                            (size_t)slot * hw->frame_stride);
    WRITE_ONCE(hdr->seq, done);
    WRITE_ONCE(hdr->len, hw->frame_slot_bytes);
    WRITE_ONCE(hdr->flags, ANLOGIC_FRAME_FLAG_COMPLETE);
    WRITE_ONCE(hdr->reserved, 0);
    dma_wmb();
    WRITE_ONCE(mb->count, done);
    dma_wmb();
    WRITE_ONCE(mb->seq, done);
    dma_wmb();
    hw->frame_push_done = done;

    pr_debug("收帧缓冲：门铃 %u，补齐槽 %u 的槽头与 mailbox.seq/count。\n",
             done, slot);
}

/*
 * 分配收帧缓冲。物理地址连续、页对齐，槽位步长也页对齐（FPGA 侧按
 * phys + stride * n 算 MWr 目标，不用再对齐）。
 * 返回 0 表示分配成功且描述符已就绪；失败返回负值（调用方走清理路径）。
 */
int anlogic_pci_frame_alloc(struct anlogic_pci_hw *hw)
{
    struct anlogic_frame_mailbox *mb;
    unsigned int width = frame_width;
    unsigned int height = frame_height;
    unsigned int slot_bytes = frame_slot_bytes;
    unsigned int slots = frame_slots;
    unsigned int hdr_bytes = sizeof(struct anlogic_frame_slot_hdr);
    unsigned int stride;
    size_t size;

    if (!hw || !hw->pdev) {
        pr_err("收帧缓冲：hw/pdev 为空。\n");
        return -EINVAL;
    }

    /* 模块参数是 root 可控的，先夹到合法区间，别让 0 槽/0 字节流下去 */
    if (!width || width > ANLOGIC_FRAME_MAX_W) {
        pr_warn("收帧缓冲：frame_width=%u 非法，改用默认 %u。\n", width,
                ANLOGIC_FRAME_DEF_WIDTH);
        width = ANLOGIC_FRAME_DEF_WIDTH;
    }
    if (!height || height > ANLOGIC_FRAME_MAX_H) {
        pr_warn("收帧缓冲：frame_height=%u 非法，改用默认 %u。\n", height,
                ANLOGIC_FRAME_DEF_HEIGHT);
        height = ANLOGIC_FRAME_DEF_HEIGHT;
    }
    if (!slot_bytes)
        slot_bytes = width * height;
    if (slot_bytes > ANLOGIC_FRAME_MAX_SLOT_BYTES) {
        pr_warn("收帧缓冲：frame_slot_bytes=%u 超过上限 %u，夹到上限。\n",
                slot_bytes, ANLOGIC_FRAME_MAX_SLOT_BYTES);
        slot_bytes = ANLOGIC_FRAME_MAX_SLOT_BYTES;
    }
    if (!slots || slots > ANLOGIC_FRAME_MAX_SLOTS) {
        pr_warn("收帧缓冲：frame_slots=%u 非法，改用 %u。\n", slots,
                ANLOGIC_FRAME_DEF_SLOTS);
        slots = ANLOGIC_FRAME_DEF_SLOTS;
    }

    /* 每槽页对齐：槽头 + 一帧像素，再向上取整到页 */
    stride = (unsigned int)PAGE_ALIGN((size_t)hdr_bytes + slot_bytes);
    /*
     * 布局（与 include/anlogic_pci_ioctl.h 的说明一致，FPGA 按这个算 MWr 目标）：
     *   [0, stride*slots)            槽区，槽 i 在 stride*i，槽头后是像素
     *   [stride*slots, +一页)         mailbox 描述符，页对齐、与槽区不重叠
     */
    size = (size_t)stride * slots + PAGE_SIZE;

    hw->frame_virt = dma_alloc_coherent(&hw->pdev->dev, size, &hw->frame_phys,
                                        GFP_KERNEL);
    if (!hw->frame_virt) {
        pr_err("收帧缓冲：dma_alloc_coherent(%zu 字节) 失败。\n", size);
        return -ENOMEM;
    }
    /* FPGA 侧按物理地址算 MWr 目标，页对齐是硬要求（DMA API 本来就保证） */
    if (hw->frame_phys & (PAGE_SIZE - 1)) {
        pr_err("收帧缓冲：物理地址 0x%llx 不是页对齐，放弃。\n",
               (unsigned long long)hw->frame_phys);
        dma_free_coherent(&hw->pdev->dev, size, hw->frame_virt,
                          hw->frame_phys);
        hw->frame_virt = NULL;
        hw->frame_phys = 0;
        return -EINVAL;
    }

    /* 清零：序号从 0 起，FPGA 没接上时用户态一问就是「无新帧」 */
    memset(hw->frame_virt, 0, size);

    /* mailbox 放在槽区之后的一整页里，与槽区不重叠（见头文件的布局说明） */
    hw->frame_mb_off = (unsigned int)((size_t)stride * slots);
    hw->frame_mb = (struct anlogic_frame_mailbox *)((u8 *)hw->frame_virt +
                                                   hw->frame_mb_off);
    hw->frame_size = size;
    hw->frame_stride = stride;
    hw->frame_slot_bytes = slot_bytes;
    hw->frame_hdr_bytes = hdr_bytes;
    hw->frame_slots = slots;
    hw->frame_width = width;
    hw->frame_height = height;

    mb = hw->frame_mb;
    mb->seq = 0;
    mb->count = 0;
    mb->push_dbell = 0;
    mb->slot_bytes = slot_bytes;
    mb->slots = slots;
    mb->stride = stride;
    mb->width = width;
    mb->height = height;
    /* magic 最后写：FPGA/用户态看到 magic 就说明描述符已经在内存里了 */
    dma_wmb();
    mb->magic = ANLOGIC_FRAME_MAGIC;
    dma_wmb();

    pr_info("收帧缓冲：phys 0x%llx, size %zu = %u 槽 × 步长 %u + mailbox 页"
            "（%ux%u, %u 字节/帧，mailbox 偏移 %zu）。\n",
            (unsigned long long)hw->frame_phys, size, slots, stride, width,
            height, slot_bytes, (size_t)stride * slots);

    return 0;
}

/* 释放收帧缓冲；没分配过或已经释放过都是空操作（出错路径可以无脑调） */
void anlogic_pci_frame_free(struct anlogic_pci_hw *hw)
{
    if (!hw || !hw->frame_virt)
        return;

    pr_debug("收帧缓冲：释放 phys 0x%llx, size %zu。\n",
             (unsigned long long)hw->frame_phys, hw->frame_size);
    dma_free_coherent(&hw->pdev->dev, hw->frame_size, hw->frame_virt,
                      hw->frame_phys);

    hw->frame_virt = NULL;
    hw->frame_mb = NULL;
    hw->frame_mb_off = 0;
    hw->frame_phys = 0;
    hw->frame_size = 0;
    hw->frame_stride = 0;
    hw->frame_slot_bytes = 0;
    hw->frame_hdr_bytes = 0;
    hw->frame_slots = 0;
    hw->frame_push_done = 0;
    hw->frame_width = 0;
    hw->frame_height = 0;
}

static void anlogic_pci_frame_fill_info(struct anlogic_pci_hw *hw,
                                        struct anlogic_ioc_frame *info)
{
    u32 seq, count;

    /* 先把 FPGA 推来的帧补进 mailbox（门铃驱动的协议补齐），再报给用户态 */
    anlogic_pci_frame_push_sync(hw);
    anlogic_pci_frame_seq_read(hw, &seq, &count);

    memset(info, 0, sizeof(*info));
    info->phys_addr = (unsigned long long)hw->frame_phys;
    info->size = (unsigned long long)hw->frame_size;
    info->stride = hw->frame_stride;
    info->slot_bytes = hw->frame_slot_bytes;
    info->hdr_bytes = hw->frame_hdr_bytes;
    info->slots = hw->frame_slots;
    info->width = hw->frame_width;
    info->height = hw->frame_height;
    info->seq = seq;
    info->count = count;
    info->mb_off = hw->frame_mb_off;
}

/*
 * 帧就绪轮询：seq 与用户手上的 last_seq 不同就算有新帧。
 * timeout_ms = 0 只查一次（纯轮询语义）；> 0 则最多等这么久，每 ANLOGIC_FRAME_POLL_MS
 * 醒一次看 mailbox。没有新帧**不是错误**：ready 保持 0 返回 0，用户态据此报「无新帧」。
 */
static long anlogic_pci_frame_wait(struct anlogic_pci_hw *hw,
                                   struct anlogic_ioc_frame_wait *arg)
{
    unsigned long deadline = jiffies + msecs_to_jiffies(arg->timeout_ms);
    u32 seq = 0, count = 0;
    long rv = 0;

    arg->ready = 0;
    for (;;) {
        /* 卸载已经开始：不要继续等（最长 600 s），尽快把锁让出去 */
        if (atomic_read(&hw->removed)) {
            rv = -ENODEV;
            break;
        }
        anlogic_pci_frame_push_sync(hw);
        anlogic_pci_frame_seq_read(hw, &seq, &count);
        if (seq != arg->last_seq) {
            arg->ready = 1;
            break;
        }
        if (!arg->timeout_ms)
            break;                      /* 只查不等 */
        if (signal_pending(current)) {
            rv = -EINTR;
            break;
        }
        if (time_after_eq(jiffies, deadline))
            break;                      /* 等超时：ready 保持 0 */
        msleep(ANLOGIC_FRAME_POLL_MS);
    }

    arg->seq = seq;
    arg->count = count;
    arg->slot = seq ? (seq - 1) % hw->frame_slots : 0;

    return rv;
}

/*
 * 帧缓冲节点的 ioctl：只认 ANLOGIC_IOCFRAMEINFO / ANLOGIC_IOCFRAMEWAIT。
 * 两条命令都只用 copy_from_user/copy_to_user，绝不解引用用户指针。
 */
long anlogic_pci_frame_ioctl(struct anlogic_pci_cdev *cdev, unsigned int cmd,
                             void __user *arg)
{
    struct anlogic_pci_hw *hw;
    int rv;

    rv = anlogic_pci_cdev_check(__func__, cdev);
    if (rv < 0)
        return rv;

    hw = cdev->hw;
    if (!hw || !hw->frame_virt || !hw->frame_slots || !hw->frame_stride) {
        pr_err("收帧缓冲没有分配，frame ioctl 不可用。\n");
        return -EINVAL;
    }

    if (_IOC_TYPE(cmd) != ANLOGIC_IOC_MAGIC) {
        pr_err("cmd %u, bad magic 0x%x/0x%x.\n", cmd, _IOC_TYPE(cmd),
               ANLOGIC_IOC_MAGIC);
        return -ENOTTY;
    }
    if (_IOC_DIR(cmd) & (_IOC_READ | _IOC_WRITE)) {
        if (!access_ok((void __user *)arg, _IOC_SIZE(cmd))) {
            pr_err("cmd %u, bad access %p.\n", cmd, (void __user *)arg);
            return -EFAULT;
        }
    }

    switch (cmd) {
    case ANLOGIC_IOCFRAMEINFO:
        {
            struct anlogic_ioc_frame info;

            anlogic_pci_frame_fill_info(hw, &info);
            if (copy_to_user((void __user *)arg, &info, sizeof(info))) {
                pr_err("copy_to_user(frame info) 失败。\n");
                return -EFAULT;
            }
        }
        break;

    case ANLOGIC_IOCFRAMEWAIT:
        {
            struct anlogic_ioc_frame_wait wait;

            if (copy_from_user((void *)&wait, (void __user *)arg,
                               sizeof(wait))) {
                pr_err("copy_from_user(frame wait) 失败。\n");
                return -EFAULT;
            }
            if (wait.timeout_ms > ANLOGIC_FRAME_MAX_WAIT_MS) {
                pr_err("frame wait: timeout_ms %u 超过上限 %u。\n",
                       wait.timeout_ms, ANLOGIC_FRAME_MAX_WAIT_MS);
                return -EINVAL;
            }
            rv = anlogic_pci_frame_wait(hw, &wait);
            if (copy_to_user((void __user *)arg, &wait, sizeof(wait))) {
                pr_err("copy_to_user(frame wait) 失败。\n");
                return -EFAULT;
            }
            return rv;
        }

    default:
        pr_err("UNKNOWN frame ioctl cmd 0x%x.\n", cmd);
        return -ENOTTY;
    }

    return 0;
}

/*
 * 把收帧缓冲映射给用户态读。用 dma_mmap_coherent：这块内存的页属性/dma 地址由
 * DMA API 统一处理（自己 remap_pfn_range 会和 DMA 属性不一致）。
 * 用户态**只读**：这是 FPGA 的 MWr 目标，不允许被用户态写坏。
 */
static int anlogic_pci_frame_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct anlogic_pci_cdev *cdev = file->private_data;
    struct anlogic_pci_hw *hw;
    unsigned long off;
    unsigned long vsize;
    int rv;

    rv = anlogic_pci_cdev_check(__func__, cdev);
    if (rv < 0)
        return rv;

    hw = cdev->hw;
    if (mutex_lock_interruptible(&hw->lock))
        return -ERESTARTSYS;
    if (atomic_read(&hw->removed)) {
        rv = -ENODEV;
        goto out_unlock;
    }

    if (!hw->frame_virt || !hw->frame_size) {
        pr_err("mmap(): 收帧缓冲没有分配。\n");
        rv = -EINVAL;
        goto out_unlock;
    }
    if (vma->vm_flags & VM_WRITE) {
        pr_err("mmap(): 收帧缓冲只允许只读映射（PROT_READ | MAP_SHARED）。\n");
        rv = -EACCES;
        goto out_unlock;
    }
    /*
     * 连 mprotect(PROT_WRITE) 也堵上：VM_MAYWRITE 置位时共享映射可以被改写成
     * 可写，而这块内存是 FPGA 的 MWr 目标，不能让用户态写进去。
     */
    vm_flags_clear(vma, VM_MAYWRITE);

    /* mmap 映射长度溢出检查：与 BAR 的 mmap 同样的溢出安全边界判断 */
    off = vma->vm_pgoff << PAGE_SHIFT;
    vsize = vma->vm_end - vma->vm_start;
    if (off >= hw->frame_size ||
        (u64)off + (u64)vsize > (u64)hw->frame_size) {
        pr_err("mmap(): off 0x%lx + vsize 0x%lx 超出收帧缓冲 0x%zx。\n",
               off, vsize, hw->frame_size);
        rv = -EINVAL;
        goto out_unlock;
    }

    /*
     * 映射完成后就不再需要 vm_ops：这块内存的引用已经由文件引用链盖住
     * （VMA 钉着 vm_file，fd 的 release 等最后一个 VMA 消失才跑），
     * 在 vm_ops->close 里再减一次计数会把引用数减成负数（见 hw.c 的说明）。
     */
    rv = dma_mmap_coherent(&hw->pdev->dev, vma, hw->frame_virt,
                           hw->frame_phys, hw->frame_size);
    if (rv) {
        pr_err("mmap(): dma_mmap_coherent 失败 %d。\n", rv);
        goto out_unlock;
    }

    rv = 0;
out_unlock:
    mutex_unlock(&hw->lock);
    return rv;
}

static long anlogic_pci_frame_fops_ioctl(struct file *filp, unsigned int cmd,
                                         unsigned long arg)
{
    struct anlogic_pci_cdev *cdev = filp->private_data;
    struct anlogic_pci_hw *hw;
    long rv;

    rv = anlogic_pci_cdev_check(__func__, cdev);
    if (rv < 0)
        return rv;

    hw = cdev->hw;
    if (mutex_lock_interruptible(&hw->lock))
        return -ERESTARTSYS;
    if (atomic_read(&hw->removed)) {
        mutex_unlock(&hw->lock);
        return -ENODEV;
    }

    rv = anlogic_pci_frame_ioctl(cdev, cmd, (void __user *)arg);

    mutex_unlock(&hw->lock);
    return rv;
}

static const struct file_operations anlogic_pci_frame_fops = {
    .owner = THIS_MODULE,
    .open = anlogic_pci_cdev_open,
    .release = anlogic_pci_cdev_close,
    .mmap = anlogic_pci_frame_mmap,
    .unlocked_ioctl = anlogic_pci_frame_fops_ioctl,
};

void anlogic_pci_frame_init(struct anlogic_pci_cdev *cdev)
{
    cdev_init(&cdev->cdev, &anlogic_pci_frame_fops);
}
