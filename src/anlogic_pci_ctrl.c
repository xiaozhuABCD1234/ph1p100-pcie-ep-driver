// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动：字符设备的文件操作层。
 *
 * 控制节点与用户 BAR 窗口节点共用这里的 file_operations：ioctl 单 dw / 批量读写、
 * 配置空间读写，以及把 BAR 映射给用户态的 mmap。
 * （收帧缓冲节点的 ioctl/mmap 在 anlogic_pci_frame.c；控制节点的 switch 里只是
 * 把两条收帧命令转发过去，方便只开一个节点也能问到缓冲物理地址与序号。）
 *
 * 本文件是所有“用户态可控 MMIO”的唯一入口，逐条实现下面这些访问规约（先验
 * MAGIC、单次不超过 4 个双字、32 位对齐、全 F 熔断交给用户态、ioctl 边界校验、
 * mmap 溢出安全的边界判断、配置空间读写用 copy_from_user/copy_to_user）。
 * 这些校验一条都不允许删除或放宽。
 */
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/pci.h>
#include <linux/uaccess.h>

#include "anlogic_pci.h"

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
 *   - off + len（字节）必须落在 pci_resource_len() 以内，用 64 位加法，
 *     以免 32 位回绕绕过上界。
 * 任何一条不满足返回 -EINVAL，0 表示可以继续访问。
 */
static int anlogic_pci_ctrl_bar_range_check(struct anlogic_pci_hw *hw,
                                            unsigned int bar_id,
                                            unsigned int off, u64 len)
{
    resource_size_t rsrc_len;

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
    rsrc_len = pci_resource_len(hw->pdev, bar_id);
    if (!rsrc_len) {
        pr_err("ioctl: bar_id %u resource length is 0.\n", bar_id);
        return -EINVAL;
    }
    if ((u64)off + len > (u64)rsrc_len) {
        pr_err("ioctl: bar%u [0x%x, +0x%llx) exceeds resource length 0x%llx.\n",
               bar_id, off, (unsigned long long)len,
               (unsigned long long)rsrc_len);
        return -EINVAL;
    }

    return 0;
}

static long anlogic_pci_ctrl_ioctl(struct file *filp, unsigned int cmd,
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
    int i;

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
        /* 校验 bar id 与范围是否合法（规约 10） */
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
        /* 校验 bar id 与范围是否合法（规约 10） */
        rv = anlogic_pci_ctrl_bar_range_check(hw, ioctl_bar.bar_id,
                                              ioctl_bar.bar_offaddr,
                                              sizeof(u32));
        if (rv < 0)
            return rv;

        addr = hw->bar[ioctl_bar.bar_id] + ioctl_bar.bar_offaddr;
        ioctl_bar.rdata = ioread32(addr);
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
        /* 校验 bar id 与范围是否合法（规约 10） */
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

        /* 规约 11：局部索引 0..len-1，bar_offaddr 只在 addr 里用一次 */
        for (i = 0; i < ioctl_bar_bulk.bar_len; i++)
            iowrite32(ioctl_bar_bulk.wdata[i], addr + (i << 2));
        break;

    case ANLOGIC_IOCBULKR:
        if (copy_from_user((void *)&ioctl_bar_bulk, (void __user *)arg,
                           sizeof(struct anlogic_ioc_bar_bulk))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
        /* 校验 bar id 与范围是否合法（规约 10） */
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

        /* 规约 11：局部索引 0..len-1，bar_offaddr 只在 addr 里用一次 */
        for (i = 0; i < ioctl_bar_bulk.bar_len; i++)
            ioctl_bar_bulk.rdata[i] = ioread32(addr + (i << 2));

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
             * 规约 13：绝不解引用用户指针；reg 限制在 4 KB 配置空间内
             * 且必须 4 字节对齐。
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
            /* 规约 13：与写路径同样的范围与对齐限制 */
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

/* 把 PCIe BAR 映射到用户态，供 mmap 使用 */
static int anlogic_pci_ctrl_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct anlogic_pci_cdev *cdev = file->private_data;
    struct anlogic_pci_hw *hw;
    unsigned long off;
    unsigned long phys;
    unsigned long vsize;
    resource_size_t rsrc_len;
    int rv;

    rv = anlogic_pci_cdev_check(__func__, cdev);
    if (rv < 0)
        return rv;

    hw = cdev->hw;

    off = vma->vm_pgoff << PAGE_SHIFT;
    if (cdev->bar < 0 || cdev->bar >= ANLOGIC_PCI_BAR_NUM) {
        pr_err("mmap(): bad bar %d.\n", cdev->bar);
        return -EINVAL;
    }
    /* 规约 12：先拒掉 off >= BAR 长度，再做任何减法 */
    rsrc_len = pci_resource_len(hw->pdev, cdev->bar);
    if (!rsrc_len || off >= rsrc_len) {
        pr_err("mmap(): off 0x%lx >= bar%d length 0x%llx.\n",
               off, cdev->bar, (unsigned long long)rsrc_len);
        return -EINVAL;
    }
    vsize = vma->vm_end - vma->vm_start;
    /*
     * 规约 12：溢出安全的比较，不用无符号相减表示剩余长度
     * （上面已保证 off < rsrc_len）。
     */
    if ((u64)off + (u64)vsize > (u64)rsrc_len) {
        pr_err("mmap(): vsize 0x%lx exceeds remaining 0x%llx of bar%d.\n",
               vsize, (unsigned long long)(rsrc_len - off), cdev->bar);
        return -EINVAL;
    }
    phys = pci_resource_start(hw->pdev, cdev->bar) + off;

    pr_debug("mmap(): cdev 0x%p, bar %d, hw 0x%p, pdev 0x%p\n", cdev,
             cdev->bar, hw, hw->pdev);
    pr_debug("off = 0x%lx, start = 0x%llx, phys = 0x%lx\n", off,
             (unsigned long long)pci_resource_start(hw->pdev, cdev->bar),
             phys);

    /*
     * MMIO 页不能按 cache line 访问 EP，所以映射成 noncached；
     * 同时防止这些页被换入换出或被 dump。
     */
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    vm_flags_set(vma, ANLOGIC_PCI_VMEM_FLAGS);

    rv = io_remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
                            vsize, vma->vm_page_prot);
    if (rv)
        return -EAGAIN;

    return 0;
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
