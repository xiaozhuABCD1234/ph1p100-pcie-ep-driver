/*
 * This file is part of the anlogic PCIe driver for Linux
 *
 * Copyright (c) 2020-present,  anlogic, Inc.
 * All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */
#include <linux/ioctl.h>
#include "version.h"
#include "anlogic_pci_drv.h"
#include "anlogic_pci_lib.h"
#include "anlogic_pci_cdev.h"
#include "cdev_ctrl.h"

#if KERNEL_VERSION(5, 0, 0) <= LINUX_VERSION_CODE
#define xlx_access_ok(X,Y,Z) access_ok(Y,Z)
#else
#define xlx_access_ok(X,Y,Z) access_ok(X,Y,Z)
#endif

extern void           *gBaseVirt ;           // Base register address (Virtual address, for I/O).
extern char           *gBufferUnaligned ;    // Pointer to Unaligned DMA buffer.
extern char           *gReadBuffer;    // Pointer to dword aligned DMA buffer.
extern char           *gWriteBuffer;    // Pointer to dword aligned DMA buffer.
extern dma_addr_t      gReadHWAddr;
extern dma_addr_t      gWriteHWAddr;

/*
 * character device file operations for control bus (through control bridge)
 */
static ssize_t char_ctrl_read(struct file *fp, char __user *buf, size_t count,
        loff_t *pos)
{
    struct anlogic_cdev *xcdev = (struct anlogic_cdev *)fp->private_data;
    struct anlogic_dev *xdev;
    void __iomem *reg;
    int i=0;
    u32 w;
    int rv;

    rv = xcdev_check(__func__, xcdev, 0);
    if (rv < 0)
        return rv;
    xdev = xcdev->xdev;

    /* only 32-bit aligned and 32-bit multiples */
    if (*pos & 3)
        return -EPROTO;
    /* first address is BAR base plus file position offset */
    reg = xdev->bar[xcdev->bar] + *pos;
    //w = read_register(reg);
   
    dbg_sg("%s(@%p, count=%ld, pos=%d) value = 0x%08x\n",
            __func__, reg, (long)count, (int)*pos, w);
    //rv = copy_to_user(buf, &w, 4);
    rv = copy_to_user(buf, (char *)gReadBuffer, count);
    for (i=0;i<count; i++) {
        dbg_sg(" value = 0x%x\n",gReadBuffer[i]);
    }
    if (rv)
        dbg_sg("Copy to userspace failed but continuing\n");

    return count;
}

static ssize_t char_ctrl_write(struct file *file, const char __user *buf,
            size_t count, loff_t *pos)
{
    struct anlogic_cdev *xcdev = (struct anlogic_cdev *)file->private_data;
    struct anlogic_dev *xdev;
    void __iomem *reg;
    u32 w;
    int rv;

    rv = xcdev_check(__func__, xcdev, 0);
    if (rv < 0)
        return rv;
    xdev = xcdev->xdev;

    /* only 32-bit aligned and 32-bit multiples */
    if (*pos & 3)
        return -EPROTO;

    /* first address is BAR base plus file position offset */
    reg = xdev->bar[xcdev->bar] + *pos;
   // rv = copy_from_user(&w, buf, 4);
    rv =  copy_from_user((char *)gWriteBuffer, buf, count);
    if (rv)
        pr_info("copy from user failed %d/4, but continuing.\n", rv);

    dbg_sg("%s(0x%08x @%p, count=%ld, pos=%d)\n",
            __func__, w, reg, (long)count, (int)*pos);
    //write_register(w, reg);
    //iowrite32(w, reg);
    
    
    //*pos += 4;
    return count;
}

static long version_ioctl(struct anlogic_cdev *xcdev, void __user *arg)
{
    struct anlogic_ioc_info obj;
    struct anlogic_dev *xdev = xcdev->xdev;
    int rv;

    rv = copy_from_user((void *)&obj, arg, sizeof(struct anlogic_ioc_info));
    if (rv) {
        pr_info("copy from user failed %d/%ld.\n",
            rv, sizeof(struct anlogic_ioc_info));
        return -EFAULT;
    }
    memset(&obj, 0, sizeof(obj));
    obj.vendor = xdev->pdev->vendor;
    obj.device = xdev->pdev->device;
    obj.subsystem_vendor = xdev->pdev->subsystem_vendor;
    obj.subsystem_device = xdev->pdev->subsystem_device;
    obj.feature_id = xdev->feature_id;
    obj.driver_version = DRV_MOD_VERSION_NUMBER;
    obj.domain = 0;
    obj.bus = PCI_BUS_NUM(xdev->pdev->devfn);
    obj.dev = PCI_SLOT(xdev->pdev->devfn);
    obj.func = PCI_FUNC(xdev->pdev->devfn);
    if (copy_to_user(arg, &obj, sizeof(struct anlogic_ioc_info)))
        return -EFAULT;
    return 0;
}

u32 XPCIe_ReadCfgReg(struct pci_dev * gDev,u32 byte) {
   u32 pciReg;
   if (pci_read_config_dword(gDev, byte, &pciReg) < 0) {
        printk("XBMD: XPCIe_ReadCfgReg: Reading PCI interface failed.");
        return (-1);
   }
   return (pciReg);
}

u32 XPCIe_WriteCfgReg(struct pci_dev * gDev,u32 byte, u32 val) {
   if (pci_write_config_dword(gDev, byte, val) < 0) {
        printk("XBMD: XPCIe_Read Device Control: Reading PCI interface failed.");
        return (-1);
   }
   return 1;
}

long char_ctrl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct anlogic_cdev *xcdev = (struct anlogic_cdev *)filp->private_data;
    struct anlogic_dev *xdev;
    struct anlogic_ioc_base ioctl_obj;
    struct anlogic_ioc_bar ioctl_bar;
    struct anlogic_ioc_bar_bulk ioctl_bar_bulk;
    long result = 0;
    void __iomem *addr;
    int rv;
    int i=0;
    uint32_t regx;

    rv = xcdev_check(__func__, xcdev, 0);
    if (rv < 0)
        return rv;

    xdev = xcdev->xdev;
    if (!xdev) {
        pr_info("cmd %u, xdev NULL.\n", cmd);
        return -EINVAL;
    }
    pr_info("cmd 0x%x, xdev 0x%p, pdev 0x%p.\n", cmd, xdev, xdev->pdev);
     
    if (_IOC_TYPE(cmd) != ANLOGIC_IOC_MAGIC) {
        pr_err("cmd %u, bad magic 0x%x/0x%x.\n",
             cmd, _IOC_TYPE(cmd), ANLOGIC_IOC_MAGIC);
        return -ENOTTY;
    }

    if (_IOC_DIR(cmd) & _IOC_READ)
        result = !xlx_access_ok(VERIFY_WRITE, (void __user *)arg,
                _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        result =  !xlx_access_ok(VERIFY_READ, (void __user *)arg,
                _IOC_SIZE(cmd));

    if (result) {
        pr_err("bad access %ld.\n", result);
        return -EFAULT;
    }

    switch (cmd) {
    case ANLOGIC_IOCINFO:
        if (copy_from_user((void *)&ioctl_obj, (void __user *) arg,
             sizeof(struct anlogic_ioc_base))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }

        if (ioctl_obj.magic != ANLOGIC_XCL_MAGIC) {
            pr_err("magic 0x%x !=  XDMA_XCL_MAGIC (0x%x).\n",
                ioctl_obj.magic, ANLOGIC_XCL_MAGIC);
            return -ENOTTY;
        }
        return version_ioctl(xcdev, (void __user *)arg);
   case ANLOGIC_IOCOFFLINE:
        //xdma_device_offline(xdev->pdev, xdev);
        break;
    case ANLOGIC_IOCONLINE:
        //xdma_device_online(xdev->pdev, xdev);
        break;
    case ANLOGIC_IOCW:  //elbi bar0 Write
        if (copy_from_user((void *)&ioctl_bar, (void __user *) arg,
             sizeof(struct anlogic_ioc_bar))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
        //to check bar id is valid and size is suitable
        addr = xdev->bar[ioctl_bar.bar_id] + ioctl_bar.bar_offaddr;  //BAR 0 base addr (virtual addr),
         
        iowrite32(ioctl_bar.wdata, addr);
         
        break;
    case ANLOGIC_IOCR:
         if (copy_from_user((void *)&ioctl_bar, (void __user *) arg,
             sizeof(struct anlogic_ioc_bar))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
        
        //to check bar id is valid and size is suitable
        addr = xdev->bar[ioctl_bar.bar_id] + ioctl_bar.bar_offaddr;  //BAR 0 base addr (virtual addr),
         
        ioctl_bar.rdata = ioread32(addr);
        if (copy_to_user((void __user *)arg, &ioctl_bar, sizeof(struct anlogic_ioc_bar))) {
            pr_err("copy_to_user failed.\n");
            return -EFAULT;
        }
         
        break;
    case ANLOGIC_IOCBULKW:
         if (copy_from_user((void *)&ioctl_bar_bulk, (void __user *) arg,
             sizeof(struct anlogic_ioc_bar_bulk))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
         //to check bar id is valid and size is suitable
        addr = xdev->bar[ioctl_bar_bulk.bar_id] + ioctl_bar_bulk.bar_offaddr;  //BAR 0 base addr (virtual addr),
        
        for (i= ioctl_bar_bulk.bar_offaddr; i< ioctl_bar_bulk.bar_len; i++) {
            iowrite32(ioctl_bar_bulk.wdata[i], addr+ (i>>2));
        }
 
        break;
   case ANLOGIC_IOCBULKR:
         if (copy_from_user((void *)&ioctl_bar_bulk, (void __user *) arg,
             sizeof(struct anlogic_ioc_bar_bulk))) {
            pr_err("copy_from_user failed.\n");
            return -EFAULT;
        }
         //to check bar id is valid and size is suitable
        addr = xdev->bar[ioctl_bar_bulk.bar_id] + ioctl_bar_bulk.bar_offaddr;  //BAR 0 base addr (virtual addr),
        
        for (i= ioctl_bar_bulk.bar_offaddr; i< ioctl_bar_bulk.bar_len; i++) {
            ioctl_bar_bulk.rdata[i] = ioread32(addr+ (i>>2));
        }        
        //ioctl_bar_bulk.rdata = ioread32(addr);
        if (copy_to_user((void __user *)arg, &ioctl_bar_bulk, sizeof(struct anlogic_ioc_bar_bulk))){
            pr_err("copy_to_user failed.\n");
            return -EFAULT;
        }
         
        break;
   case ANLOGIC_IOCCFGW:  //PCI CFG WRITE
        regx = XPCIe_WriteCfgReg(xdev->pdev,(*(cfgwr *)arg).reg,(*(cfgwr *)arg).value);
        printk(KERN_WARNING"%d: Write Register.\n", (*(cfgwr *)arg).reg);
        printk(KERN_WARNING"%d: Write Value\n", (*(cfgwr *)arg).value);
        break;
   case ANLOGIC_IOCCFGR:  //PCI CFG READ
        regx = XPCIe_ReadCfgReg(xdev->pdev,*(uint32_t *)arg);  
        *((uint32_t *)arg) = regx;
      break;
    default:
        pr_err("UNKNOWN ioctl cmd 0x%x.\n", cmd);
        return -ENOTTY;
    }
    return 0;
}

/* maps the PCIe BAR into user space for memory-like access using mmap() */
int bridge_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct anlogic_dev *xdev;
    struct anlogic_cdev *xcdev = (struct anlogic_cdev *)file->private_data;
    unsigned long off;
    unsigned long phys;
    unsigned long vsize;
    unsigned long psize;
    int rv;

    rv = xcdev_check(__func__, xcdev, 0);
    if (rv < 0)
        return rv;
    xdev = xcdev->xdev;

    off = vma->vm_pgoff << PAGE_SHIFT;
    /* BAR physical address */
    phys = pci_resource_start(xdev->pdev, xcdev->bar) + off;
    vsize = vma->vm_end - vma->vm_start;
    /* complete resource */
    psize = pci_resource_end(xdev->pdev, xcdev->bar) -
        pci_resource_start(xdev->pdev, xcdev->bar) + 1 - off;

    dbg_sg("mmap(): xcdev = 0x%08lx\n", (unsigned long)xcdev);
    dbg_sg("mmap(): cdev->bar = %d\n", xcdev->bar);
    dbg_sg("mmap(): xdev = 0x%p\n", xdev);
    dbg_sg("mmap(): pci_dev = 0x%08lx\n", (unsigned long)xdev->pdev);

    dbg_sg("off = 0x%lx\n", off);
    dbg_sg("start = 0x%llx\n",
        (unsigned long long)pci_resource_start(xdev->pdev,
        xcdev->bar));
    dbg_sg("phys = 0x%lx\n", phys);

    if (vsize > psize)
        return -EINVAL;
    /*
     * pages must not be cached as this would result in cache line sized
     * accesses to the end point
     */
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    /*
     * prevent touching the pages (byte access) for swap-in,
     * and prevent the pages from being swapped out
     */
    vm_flags_set(vma, VMEM_FLAGS);   /* 6.3+ vm_flags 只读，改用 vm_flags_set() */
    /* make MMIO accessible to user space */
    rv = io_remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
            vsize, vma->vm_page_prot);
    dbg_sg("vma=0x%p, vma->vm_start=0x%lx, phys=0x%lx, size=%lu = %d\n",
        vma, vma->vm_start, phys >> PAGE_SHIFT, vsize, rv);

    if (rv)
        return -EAGAIN;
    return 0;
}

/*
 * character device file operations for control bus (through control bridge)
 */
static const struct file_operations ctrl_fops = {
    .owner = THIS_MODULE,
    .open = char_open,
    .release = char_close,
    .read = char_ctrl_read,
    .write = char_ctrl_write,
    .mmap = bridge_mmap,
    .unlocked_ioctl = char_ctrl_ioctl,
};

void cdev_ctrl_init(struct anlogic_cdev *xcdev)
{
    cdev_init(&xcdev->cdev, &ctrl_fops);
}
