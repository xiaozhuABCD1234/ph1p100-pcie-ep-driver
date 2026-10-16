// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动：字符设备的文件操作层。
 *
 * 控制节点与用户 BAR 窗口节点共用这里的 file_operations：ioctl 单 dw / 批量读写、
 * 配置空间读写，以及把 BAR 映射给用户态的 mmap。
 * （收帧缓冲节点的 ioctl/mmap 在 anlogic_pci_frame.c；控制节点的 switch 里只是
 * 把两条收帧命令转发过去，方便只开一个节点也能问到缓冲物理地址与序号。）
 *
 * 本文件是控制/用户 BAR 节点上用户态可控 MMIO 的入口。ioctl 这条路径逐条实现
 * 下面这些访问校验（先读 BAR0+0x40 验 MAGIC、单笔不超过 4 个双字、读到
 * 0xffffffff 立即熔断返回 -EIO、32 位对齐、ioctl 边界校验、配置空间读写用
 * copy_from_user/copy_to_user）；这些校验一条都不允许删除或放宽。
 *
 * /dev/..._user 的 mmap 是一条裸窗口：驱动只保证「offset 0、一页、落在已登记的
 * 控制/用户 BAR 内」，逐 offset 白名单、先验 MAGIC 与全 F 熔断由用户态自持
 * （app/rs 的 cam_grab 甚至刻意绕过驱动、直接 mmap sysfs resource0）。收帧
 * 节点的 mmap 映射的是驱动自己分配的收帧内存，不走这里。
 *
 * 生命周期：ioctl/mmap 入口都用 hw->lock 串行化，并在 remove 置 removed 之后
 * 拒绝执行（见 anlogic_pci_hw_get / anlogic_pci_hw_quiesce）。
 */
#include <linux/ioctl.h>
#include <linux/minmax.h>
#include <linux/mm.h>
#include <linux/pci.h>
#include <linux/uaccess.h>

#include "anlogic_pci.h"

/* 一次连续 MMIO 访问的双字上限（超过就分块），与访问规约一致 */
#define ANLOGIC_CTRL_MMIO_CHUNK_DW  4u

static u32 anlogic_pci_read_cfg_dword(struct pci_dev *pdev, u32 off)
{
    u32 val;

    if (pci_read_config_dword(pdev, off, &val) < 0) {
        pr_err("reading PCI config 0x%x failed.\n", off);
        return (u32)-1;
    }

    return val;
}

static u32 anlogic_pci_write_cfg_dword(struct pci_dev *pdev, u32 off, u32 val)
{
    if (pci_write_config_dword(pdev, off, val) < 0) {
        pr_err("writing PCI config 0x%x failed.\n", off);
        return (u32)-1;
    }

    return 1;
}

static long anlogic_pci_version_ioctl(struct anlogic_pci_cdev *cdev,
                                      void __user *arg)
{
    struct anlogic_ioc_info obj;
    struct anlogic_pci_hw *hw = cdev->hw;

    if (copy_from_user((void *)&obj, arg, sizeof(struct anlogic_ioc_info))) {
        pr_err("copy_from_user failed %ld.\n",
               sizeof(struct anlogic_ioc_info));
        return -EFAULT;
    }
    memset(&obj, 0, sizeof(obj));
    obj.vendor = hw->pdev->vendor;
    obj.device = hw->pdev->device;
    obj.subsystem_vendor = hw->pdev->subsystem_vendor;
    obj.subsystem_device = hw->pdev->subsystem_device;
    obj.feature_id = hw->feature_id;
    obj.driver_version = ANLOGIC_PCI_DRV_VERSION_NUMBER;
    obj.domain = 0;
    obj.bus = PCI_BUS_NUM(hw->pdev->devfn);
    obj.dev = PCI_SLOT(hw->pdev->devfn);
    obj.func = PCI_FUNC(hw->pdev->devfn);
    if (copy_to_user(arg, &obj, sizeof(struct anlogic_ioc_info)))
        return -EFAULT;

    return 0;
}

/*
 * 所有 BAR ioctl 的公共入口校验（本文件的公共校验）：
 *   - bar_id 必须 < ANLOGIC_PCI_BAR_NUM 且该 BAR 确实映射过；
 *   - off（BAR 内字节偏移）必须 4 字节对齐；
 *   - off + len（字节）必须落在该 BAR 实际映射长度（hw->bar_map_len）以内，
 *     用 64 位加法，以免 32 位回绕绕过上界。
 * 任何一条不满足返回 -EINVAL，0 表示可以继续访问。
 */
static int anlogic_pci_ctrl_bar_range_check(struct anlogic_pci_hw *hw,
                                            unsigned int bar_id,
                                            unsigned int off, u64 len)
{
    resource_size_t map_len;

    if (bar_id >= ANLOGIC_PCI_BAR_NUM) {
        pr_err("ioctl: bar_id %u >= ANLOGIC_PCI_BAR_NUM %d.\n", bar_id,
               ANLOGIC_PCI_BAR_NUM);
        return -EINVAL;
    }
    if (!hw->bar[bar_id]) {
        pr_err("ioctl: bar_id %u is not mapped.\n", bar_id);
        return -EINVAL;
    }
    if (off & 3) {
        pr_err("ioctl: bar_offaddr 0x%x is not 4-byte aligned.\n", off);
        return -EINVAL;
    }
    /*
     * 上界用实际 ioremap 出来的长度，不用 pci_resource_len()：BAR 超过 INT_MAX
     * 时映射会被夹短，用资源长度会放过跑到映射外面的虚拟地址。
     */
    map_len = hw->bar_map_len[bar_id];
    if (!map_len) {
        pr_err("ioctl: bar_id %u mapped length is 0.\n", bar_id);
        return -EINVAL;
    }
    if ((u64)off + len > (u64)map_len) {
        pr_err("ioctl: bar%u [0x%x, +0x%llx) exceeds mapped length 0x%llx.\n",
               bar_id, off, (unsigned long long)len,
               (unsigned long long)map_len);
        return -EINVAL;
    }

    return 0;
}

/*
 * 一次 BAR 读的统一实现（三条门禁里的两条与另一半）：
 *   - BAR0（寄存器窗）上任何一次读之前先读 0x40 验 MAGIC，不符就不碰目标地址；
 *   - 读回 0xffffffff 立即熔断，向外返回 -EIO。
 * MAGIC 口只有 0x40 这个无副作用口，0x60 数据口会动 PTR，所以只在这里读 0x40。
 */
static int anlogic_pci_ctrl_read32(struct anlogic_pci_hw *hw,
                                   unsigned int bar_id, unsigned int off,
                                   u32 *val)
{
    if ((int)bar_id == hw->config_bar_idx &&
        hw->bar_map_len[bar_id] >=
                ANLOGIC_BAR_MAGIC_OFF + sizeof(u32)) {
        u32 magic = ioread32(hw->bar[bar_id] + ANLOGIC_BAR_MAGIC_OFF);

        if (magic != ANLOGIC_BAR_MAGIC_VAL) {
            pr_err("ioctl: bar%u MAGIC 0x%08x != 0x%08x, refuse read of 0x%x.\n",
                   bar_id, magic, ANLOGIC_BAR_MAGIC_VAL, off);
            return -EIO;
        }
    }

    *val = ioread32(hw->bar[bar_id] + off);
    if (*val == ANLOGIC_MMIO_ALL_F) {
        pr_err("ioctl: bar%u+0x%x reads 0xffffffff, device is dead, fuse.\n",
               bar_id, off);
        return -EIO;
    }

    return 0;
}

static long anlogic_pci_ctrl_ioctl_body(struct file *filp, unsigned int cmd,
                                        unsigned long arg)
{
    struct anlogic_pci_cdev *cdev = filp->private_data;
    struct anlogic_pci_hw *hw;
    struct anlogic_ioc_base ioctl_obj;
    struct anlogic_ioc_bar ioctl_bar;
    struct anlogic_ioc_bar_bulk ioctl_bar_bulk;
    long result = 0;
    void __iomem *addr;
    int rv;
    unsigned int i;

    rv = anlogic_pci_cdev_check(__func__, cdev);
    if (rv < 0)
        return rv;

    hw = cdev->hw;
    if (!hw) {
        pr_info("cmd %u, hw NULL.\n", cmd);
        return -EINVAL;
    }
    pr_debug("cmd 0x%x, hw 0x%p, pdev 0x%p.\n", cmd, hw, hw->pdev);

    if (_IOC_TYPE(cmd) != ANLOGIC_IOC_MAGIC) {
        pr_err("cmd %u, bad magic 0x%x/0x%x.\n", cmd, _IOC_TYPE(cmd),
               ANLOGIC_IOC_MAGIC);
        return -ENOTTY;
    }

    if (_IOC_DIR(cmd) & _IOC_READ)
        result = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        result = !access_ok((void __user *)arg, _IOC_SIZE(cmd));

    if (result) {
        pr_err("bad access %ld.\n", result);
        return -EFAULT;
    }

    switch (cmd) {
    case ANLOGIC_IOCINFO:
        if (copy_from_user((void *)&ioctl_obj, (void __user *)arg,
                           sizeof(struct anlogic_ioc_base))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
        if (ioctl_obj.magic != ANLOGIC_XCL_MAGIC) {
            pr_err("magic 0x%x != ANLOGIC_XCL_MAGIC (0x%x).\n",
                   ioctl_obj.magic, ANLOGIC_XCL_MAGIC);
            return -ENOTTY;
        }
        return anlogic_pci_version_ioctl(cdev, (void __user *)arg);

    /* 线下/上线没有实现，保留编号以免破坏用户态兼容 */
    case ANLOGIC_IOCOFFLINE:
    case ANLOGIC_IOCONLINE:
        break;

    case ANLOGIC_IOCW:
        if (copy_from_user((void *)&ioctl_bar, (void __user *)arg,
                           sizeof(struct anlogic_ioc_bar))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
        /* 写入前先校验：bar id 合法、bar_offaddr 4 字节对齐、
         * 这一个双字不越 BAR 边界 */
        rv = anlogic_pci_ctrl_bar_range_check(hw, ioctl_bar.bar_id,
                                              ioctl_bar.bar_offaddr,
                                              sizeof(u32));
        if (rv < 0)
            return rv;

        addr = hw->bar[ioctl_bar.bar_id] + ioctl_bar.bar_offaddr;
        iowrite32(ioctl_bar.wdata, addr);
        break;

    case ANLOGIC_IOCR:
        if (copy_from_user((void *)&ioctl_bar, (void __user *)arg,
                           sizeof(struct anlogic_ioc_bar))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
        /* 读取前先校验：bar id 合法、bar_offaddr 4 字节对齐、
         * 这一个双字不越 BAR 边界 */
        rv = anlogic_pci_ctrl_bar_range_check(hw, ioctl_bar.bar_id,
                                              ioctl_bar.bar_offaddr,
                                              sizeof(u32));
        if (rv < 0)
            return rv;

        rv = anlogic_pci_ctrl_read32(hw, ioctl_bar.bar_id,
                                     ioctl_bar.bar_offaddr,
                                     &ioctl_bar.rdata);
        if (rv < 0)
            return rv;
        if (copy_to_user((void __user *)arg, &ioctl_bar,
                         sizeof(struct anlogic_ioc_bar))) {
            pr_err("copy_to_user failed.\n");
            return -EFAULT;
        }
        break;

    case ANLOGIC_IOCBULKW:
        if (copy_from_user((void *)&ioctl_bar_bulk, (void __user *)arg,
                           sizeof(struct anlogic_ioc_bar_bulk))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
        /* 批量写前先校验：bar id 合法、bar_offaddr 4 字节对齐、
         * 整段 bar_len 个双字不越 BAR 边界 */
        if (ioctl_bar_bulk.bar_len > MAX_DATA_SIZE) {
            pr_err("bar_len %u > MAX_DATA_SIZE %d.\n",
                   ioctl_bar_bulk.bar_len, MAX_DATA_SIZE);
            return -EINVAL;
        }
        rv = anlogic_pci_ctrl_bar_range_check(hw, ioctl_bar_bulk.bar_id,
                                              ioctl_bar_bulk.bar_offaddr,
                                              (u64)ioctl_bar_bulk.bar_len *
                                              sizeof(u32));
        if (rv < 0)
            return rv;

        addr = hw->bar[ioctl_bar_bulk.bar_id] + ioctl_bar_bulk.bar_offaddr;

        /*
         * 按 ANLOGIC_CTRL_MMIO_CHUNK_DW 个双字一块推进：单笔不越过 4 dw；
         * bar_offaddr 只在算基址 addr 时用一次，块内用局部下标。
         */
        for (i = 0; i < ioctl_bar_bulk.bar_len;
             i += ANLOGIC_CTRL_MMIO_CHUNK_DW) {
            unsigned int n = min_t(unsigned int, ANLOGIC_CTRL_MMIO_CHUNK_DW,
                                   ioctl_bar_bulk.bar_len - i);
            unsigned int k;

            for (k = 0; k < n; k++)
                iowrite32(ioctl_bar_bulk.wdata[i + k],
                          addr + ((i + k) << 2));
        }
        break;

    case ANLOGIC_IOCBULKR:
        if (copy_from_user((void *)&ioctl_bar_bulk, (void __user *)arg,
                           sizeof(struct anlogic_ioc_bar_bulk))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
        /* 批量读前先校验：bar id 合法、bar_offaddr 4 字节对齐、
         * 整段 bar_len 个双字不越 BAR 边界 */
        if (ioctl_bar_bulk.bar_len > MAX_DATA_SIZE) {
            pr_err("bar_len %u > MAX_DATA_SIZE %d.\n",
                   ioctl_bar_bulk.bar_len, MAX_DATA_SIZE);
            return -EINVAL;
        }
        rv = anlogic_pci_ctrl_bar_range_check(hw, ioctl_bar_bulk.bar_id,
                                              ioctl_bar_bulk.bar_offaddr,
                                              (u64)ioctl_bar_bulk.bar_len *
                                              sizeof(u32));
        if (rv < 0)
            return rv;

        /*
         * 按 ANLOGIC_CTRL_MMIO_CHUNK_DW 个双字一块读，每一笔都走带 MAGIC 门禁与
         * 全 F 熔断的 anlogic_pci_ctrl_read32；块内用局部下标。
         */
        for (i = 0; i < ioctl_bar_bulk.bar_len;
             i += ANLOGIC_CTRL_MMIO_CHUNK_DW) {
            unsigned int n = min_t(unsigned int, ANLOGIC_CTRL_MMIO_CHUNK_DW,
                                   ioctl_bar_bulk.bar_len - i);
            unsigned int k;

            for (k = 0; k < n; k++) {
                rv = anlogic_pci_ctrl_read32(hw, ioctl_bar_bulk.bar_id,
                                             ioctl_bar_bulk.bar_offaddr +
                                             ((i + k) << 2),
                                             &ioctl_bar_bulk.rdata[i + k]);
                if (rv < 0)
                    return rv;
            }
        }

        if (copy_to_user((void __user *)arg, &ioctl_bar_bulk,
                         sizeof(struct anlogic_ioc_bar_bulk))) {
            pr_err("copy_to_user failed.\n");
            return -EFAULT;
        }
        break;

    case ANLOGIC_IOCCFGW:
        {
            cfgwr cfg;

            if (copy_from_user((void *)&cfg, (void __user *)arg,
                               sizeof(cfg))) {
                pr_err("copy_from_user failed.\n");
                return -EFAULT;
            }
            /*
             * 配置空间写：reg 必须落在 4 KB 配置空间内且 4 字节对齐，
             * 只把整数偏移交给 pci_write_config_dword，绝不解引用用户指针。
             */
            if (cfg.reg < 0 || (cfg.reg & 3) || cfg.reg >= 0x1000) {
                pr_err("bad config offset 0x%x.\n", cfg.reg);
                return -EINVAL;
            }
            anlogic_pci_write_cfg_dword(hw->pdev, (u32)cfg.reg,
                                        (u32)cfg.value);
            pr_debug("cfg write: reg 0x%x = 0x%x.\n", cfg.reg, cfg.value);
        }
        break;

    case ANLOGIC_IOCCFGR:
        {
            u32 cfg_off;
            u32 regx;

            if (copy_from_user((void *)&cfg_off, (void __user *)arg,
                               sizeof(cfg_off))) {
                pr_err("copy_from_user failed.\n");
                return -EFAULT;
            }
            /* 配置空间读：与写路径同样的范围与对齐限制 */
            if ((cfg_off & 3) || cfg_off >= 0x1000) {
                pr_err("bad config offset 0x%x.\n", cfg_off);
                return -EINVAL;
            }
            regx = anlogic_pci_read_cfg_dword(hw->pdev, cfg_off);
            if (copy_to_user((void __user *)arg, &regx, sizeof(regx))) {
                pr_err("copy_to_user failed.\n");
                return -EFAULT;
            }
        }
        break;

    /*
     * 收帧缓冲的命令也允许从控制节点发（两条命令的实现在 anlogic_pci_frame.c，
     * 本节点只做转发；上面的类型/access_ok 校验照旧先跑一遍）。
     * 帧数据本身在 /dev/ANLOGIC-PCI0_frame 上只读 mmap 取。
     */
    case ANLOGIC_IOCFRAMEINFO:
    case ANLOGIC_IOCFRAMEWAIT:
        return anlogic_pci_frame_ioctl(cdev, cmd, (void __user *)arg);

    default:
        pr_err("UNKNOWN ioctl cmd 0x%x.\n", cmd);
        return -ENOTTY;
    }

    return 0;
}

/* 把 PCIe BAR 映射到用户态，供 mmap 使用（只放 offset 0 的一页窗口） */
static int anlogic_pci_ctrl_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct anlogic_pci_cdev *cdev = file->private_data;
    struct anlogic_pci_hw *hw;
    unsigned long phys;
    unsigned long vsize;
    resource_size_t map_len;
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

    /* 只放登记过的控制/用户 BAR，其他 BAR 一律拒绝 */
    if (cdev->bar != hw->config_bar_idx && cdev->bar != hw->user_bar_idx) {
        pr_err("mmap(): bar %d is not a registered control/user BAR.\n",
               cdev->bar);
        rv = -EINVAL;
        goto out_unlock;
    }
    /*
     * 本节点是给用户态的裸 4 KB 窗口：只允许 offset 0、长度不超过一页。
     * 逐 offset 白名单、先验 MAGIC 与全 F 熔断在这条路径上由用户态自持
     * （cam_grab 就是刻意直接 mmap sysfs resource0 的同类做法）。
     */
    if (vma->vm_pgoff || (vma->vm_end - vma->vm_start) > PAGE_SIZE) {
        pr_err("mmap(): only offset 0 and at most one page (4 KB) allowed.\n");
        rv = -EINVAL;
        goto out_unlock;
    }

    map_len = hw->bar_map_len[cdev->bar];
    vsize = vma->vm_end - vma->vm_start;
    if (!map_len || (u64)vsize > (u64)map_len) {
        pr_err("mmap(): vsize 0x%lx exceeds bar%d mapped length 0x%llx.\n",
               vsize, cdev->bar, (unsigned long long)map_len);
        rv = -EINVAL;
        goto out_unlock;
    }
    phys = pci_resource_start(hw->pdev, cdev->bar);

    pr_debug("mmap(): cdev 0x%p, bar %d, hw 0x%p, pdev 0x%p\n", cdev,
             cdev->bar, hw, hw->pdev);
    pr_debug("off = 0x0, start = 0x%llx, phys = 0x%lx\n",
             (unsigned long long)pci_resource_start(hw->pdev, cdev->bar),
             phys);

    /*
     * MMIO 页不能按 cache line 访问 EP，所以映射成 noncached；
     * 同时防止这些页被换入换出或被 dump。
     * vm_ops 的 open/close 计引用，卸载时等这条映射关闭。
     */
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    vm_flags_set(vma, ANLOGIC_PCI_VMEM_FLAGS);
    vma->vm_ops = &anlogic_pci_vm_ops;
    vma->vm_private_data = hw;

    rv = io_remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
                            vsize, vma->vm_page_prot);
    if (rv)
        rv = -EAGAIN;

out_unlock:
    mutex_unlock(&hw->lock);
    return rv;
}

/*
 * ioctl 入口：hw->lock 把同一实例上的 ioctl 与 remove 串行化，removed 置位后一律
 * 拒绝。等帧最长 600 s 也在这把锁下，但等帧循环会看 removed 提前退出，所以 remove
 * 不会被拖满 600 s。
 */
static long anlogic_pci_ctrl_ioctl(struct file *filp, unsigned int cmd,
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

    rv = anlogic_pci_ctrl_ioctl_body(filp, cmd, arg);

    mutex_unlock(&hw->lock);
    return rv;
}

static const struct file_operations anlogic_pci_ctrl_fops = {
    .owner = THIS_MODULE,
    .open = anlogic_pci_cdev_open,
    .release = anlogic_pci_cdev_close,
    .mmap = anlogic_pci_ctrl_mmap,
    .unlocked_ioctl = anlogic_pci_ctrl_ioctl,
};

void anlogic_pci_ctrl_init(struct anlogic_pci_cdev *cdev)
{
    cdev_init(&cdev->cdev, &anlogic_pci_ctrl_fops);
}
