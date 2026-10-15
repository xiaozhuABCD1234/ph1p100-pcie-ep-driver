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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>

#include "anlogic_pci_lib.h"

/*
static inline void xdev_list_add(struct anlogic_pci_dev *xdev)
{
    mutex_lock(&xdev_mutex);
    if (list_empty(&xdev_list))
        xdev->idx = 0;
    else {
        struct anlogic_pci_dev *last;

        last = list_last_entry(&xdev_list, struct xdma_dev, list_head);
        xdev->idx = last->idx + 1;
    }
    list_add_tail(&xdev->list_head, &xdev_list);
    mutex_unlock(&xdev_mutex);

    dbg_init("dev %s, xdev 0x%p, xdma idx %d.\n",
         dev_name(&xdev->pdev->dev), xdev, xdev->idx);

    spin_lock(&xdev_rcu_lock);
    list_add_tail_rcu(&xdev->rcu_node, &xdev_rcu_list);
    spin_unlock(&xdev_rcu_lock);
}
*/
/*
static int is_config_bar(struct anlogic_pci_dev *xdev, int idx)
{
    u32 irq_id = 0;
    u32 cfg_id = 0;
    int flag = 0;
    u32 mask = 0xffff0000;
    struct interrupt_regs *irq_regs =
        (struct interrupt_regs *)(xdev->bar[idx] + XDMA_OFS_INT_CTRL);
    struct config_regs *cfg_regs =
        (struct config_regs *)(xdev->bar[idx] + XDMA_OFS_CONFIG);

    irq_id = read_register(&irq_regs->identifier);
    cfg_id = read_register(&cfg_regs->identifier);

    if (((irq_id & mask) == IRQ_BLOCK_ID) &&
        ((cfg_id & mask) == CONFIG_BLOCK_ID)) {
        dbg_init("BAR %d is the XDMA config BAR\n", idx);
        flag = 1;
    } else {
        dbg_init("BAR %d is NOT the XDMA config BAR: 0x%x, 0x%x.\n",
             idx, irq_id, cfg_id);
        flag = 0;
    }

    return flag;
}
*/

/*
 * MSI-X interrupt:
 *  <h2c+c2h channel_max> vectors, followed by <user_max> vectors
 */

/*
 * RTO - code to detect if MSI/MSI-X capability exists is derived
 * from linux/pci/msi.c - pci_msi_check_device
 */
static unsigned int interrupt_mode = 1;   /* 1 = MSI（本 EP 实现 MSI，无 MSI-X）；0=MSI-X 2=Legacy */

void           *gBaseVirt = NULL;           // Base register address (Virtual address, for I/O).
char           *gBufferUnaligned = NULL;    // Pointer to Unaligned DMA buffer.
char           *gReadBuffer      = NULL;    // Pointer to dword aligned DMA buffer.
char           *gWriteBuffer     = NULL;    // Pointer to dword aligned DMA buffer.
dma_addr_t      gReadHWAddr;
dma_addr_t      gWriteHWAddr;
unsigned long SA_SHIRQ = 0;

static irqreturn_t anlogic_channel_irq(int irq, void *dev_id);
static irqreturn_t anlogic_user_irq(int irq, void *dev_id);
static irqreturn_t anlogic_isr(int irq, void *dev_id);
module_param(interrupt_mode, uint, 0644);
MODULE_PARM_DESC(interrupt_mode, "0 - MSI-x , 1 - MSI, 2 - Legacy");
#ifndef arch_msi_check_device
static int arch_msi_check_device(struct pci_dev *dev, int nvec, int type)
{
    return 0;
}
#endif

u32 XPCIe_ReadReg (u32 dw_offset)
{
        u32 ret = 0;
        //u32 reg_addr = (u32)(gBaseVirt + (4 * dw_offset));
        //ret = readl(reg_addr);  
        ret = readl(gBaseVirt + (4 * dw_offset));  

        return ret; 
}

void XPCIe_WriteReg (u32 dw_offset, u32 val)
{
        //u32 reg_addr = (u32)(gBaseVirt + (4 * dw_offset));
        writel(val, (gBaseVirt + (4 * dw_offset)));
}


static void XPCIe_exit(struct pci_dev *pdev)
{


    // Free Write and Read buffers allocated to use
    /* 注：gReadBuffer/gWriteBuffer 在本驱动里从未分配过（厂商遗留 XBMD 代码），
       原先这里先 kfree 再 free_consistent 属于双重释放，已去掉 kfree。 */
    // Free memory allocated to our Endpoint
    dma_free_coherent(&pdev->dev, BUF_SIZE, gReadBuffer, gReadHWAddr);
    dma_free_coherent(&pdev->dev, BUF_SIZE, gWriteBuffer, gWriteHWAddr);

    gReadBuffer = NULL;
    gWriteBuffer = NULL;
    // Update Kernel log stating driver is unloaded
   // printk(KERN_ALERT"%s driver is unloaded\n", gDrvrName);
}


//--- XPCIe_InitCard(): Initializes XBMD descriptor registers to default values
//--- Arguments: None
//--- Return Value: None
//--- Detailed Description: 1) Resets device
//---                       2) Writes specific values into the XBMD registers inside the EP 
void XPCIe_InitCard(void)
{

  XPCIe_WriteReg(0, 1);                   // Write: DCSR (offset 0) with value of 1 (Reset Device)
  XPCIe_WriteReg(0, 0);                   // Write: DCSR (offset 0) with value of 0 (Make Active)

  XPCIe_WriteReg(2, gWriteHWAddr);        // Write: Write DMA TLP Address register with starting address
  XPCIe_WriteReg(3, 0x20);                // Write: Write DMA TLP Size register with default value (32dwords)
  XPCIe_WriteReg(4, 0x2000);              // Write: Write DMA TLP Count register with default value (2000)
  XPCIe_WriteReg(5, 0x00000000);          // Write: Write DMA TLP Pattern register with default value (0x0)

  XPCIe_WriteReg(6, 0xfeedbeef);          // Write: Read DMA Expected Data Pattern with default value (feedbeef)
  XPCIe_WriteReg(7, gReadHWAddr);         // Write: Read DMA TLP Address register with starting address.
  XPCIe_WriteReg(8, 0x20);                // Write: Read DMA TLP Size register with default value (32dwords)
  XPCIe_WriteReg(9, 0x2000);              // Write: Read DMA TLP Count register with default value (2000)
}



/* type = PCI_CAP_ID_MSI or PCI_CAP_ID_MSIX */
static int msi_msix_capable(struct pci_dev *dev, int type)
{
    struct pci_bus *bus;
    int ret;

    if (!dev || dev->no_msi)
        return 0;

    for (bus = dev->bus; bus; bus = bus->parent)
        if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
            return 0;

    ret = arch_msi_check_device(dev, 1, type);
    if (ret)
        return 0;

    if (!pci_find_capability(dev, type))
        return 0;

    return 1;
}

static void disable_msi_msix(struct anlogic_dev *xdev, struct pci_dev *pdev)
{
    if (xdev->msix_enabled) {
        pci_disable_msix(pdev);
        xdev->msix_enabled = 0;
    } else if (xdev->msi_enabled) {
        pci_disable_msi(pdev);
        xdev->msi_enabled = 0;
    }
}

static int enable_msi_msix(struct anlogic_dev *xdev, struct pci_dev *pdev)
{
    int rv = 0;

    if (!xdev) {
        pr_err("Invalid xdev\n");
        return -EINVAL;
    }

    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    if (!interrupt_mode && msi_msix_capable(pdev, PCI_CAP_ID_MSIX)) {
    
		/*
    int req_nvec = xdev->c2h_channel_max + xdev->h2c_channel_max +
                   xdev->user_max;
				   */
		int req_nvec = 8;
                   

#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        dbg_init("Enabling MSI-X\n");
        rv = pci_alloc_irq_vectors(pdev, req_nvec, req_nvec,
                       PCI_IRQ_MSIX);
      dbg_init("PCI alloc irq: %d vector\n ",rv);
#else
        int i;

        dbg_init("Enabling MSI-X\n");
        for (i = 0; i < req_nvec; i++)
            xdev->entry[i].entry = i;   //linux max support 32 msix vector

        rv = pci_enable_msix(pdev, xdev->entry, req_nvec);
#endif
        if (rv < 0)
            dbg_init("Couldn't enable MSI-X mode: %d\n", rv);

        xdev->msix_enabled = 1;

    } else if (interrupt_mode == 1 &&
           msi_msix_capable(pdev, PCI_CAP_ID_MSI)) {
        /* enable message signalled interrupts */
        dbg_init("pci_enable_msi()\n");
        rv = pci_enable_msi(pdev);
        if (rv < 0)
            dbg_init("Couldn't enable MSI mode: %d\n", rv);
        xdev->msi_enabled = 1;

    } else {
        dbg_init("MSI/MSI-X not detected - using legacy interrupts\n");
    }

    return rv;
}

static void pci_check_intr_pend(struct pci_dev *pdev)
{
    u16 v;

    pci_read_config_word(pdev, PCI_STATUS, &v);
    if (v & PCI_STATUS_INTERRUPT) {
        pr_info("%s PCI STATUS Interrupt pending 0x%x.\n",
            dev_name(&pdev->dev), v);
        pci_write_config_word(pdev, PCI_STATUS, PCI_STATUS_INTERRUPT);
    }
}


static void pci_keep_intx_enabled(struct pci_dev *pdev)
{
    /* workaround to a h/w bug:
     * when msix/msi become unavaile, default to legacy.
     * However the legacy enable was not checked.
     * If the legacy was disabled, no ack then everything stuck
     */
    u16 pcmd, pcmd_new;

    pci_read_config_word(pdev, PCI_COMMAND, &pcmd);
    pcmd_new = pcmd & ~PCI_COMMAND_INTX_DISABLE;
    if (pcmd_new != pcmd) {
        pr_info("%s: clear INTX_DISABLE, 0x%x -> 0x%x.\n",
            dev_name(&pdev->dev), pcmd, pcmd_new);
        pci_write_config_word(pdev, PCI_COMMAND, pcmd_new);
    }
}

static void prog_irq_msix_user(struct anlogic_dev *xdev, bool clear)
{
    
    struct interrupt_regs *int_regs =
        (struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
                      XDMA_OFS_INT_CTRL);
    u32 i = xdev->c2h_channel_max + xdev->h2c_channel_max;
    u32 max = i + xdev->user_max;
    int j;
    /*
    for (j = 0; i < max; j++) {
        u32 val = 0;
        int k;
        int shift = 0;

        if (clear)
            i += 4;
        else
            for (k = 0; k < 4 && i < max; i++, k++, shift += 8)
                val |= (i & 0x1f) << shift;

        write_register(
            val, &int_regs->user_msi_vector[j],
            XDMA_OFS_INT_CTRL +
                ((unsigned long)&int_regs->user_msi_vector[j] -
                 (unsigned long)int_regs));

        dbg_init("vector %d, 0x%x.\n", j, val);
    }
    */
}

static void prog_irq_msix_channel(struct anlogic_dev *xdev, bool clear)
{
    
    struct interrupt_regs *int_regs =
        (struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
                      XDMA_OFS_INT_CTRL);
    u32 max = xdev->c2h_channel_max + xdev->h2c_channel_max;
    u32 i;
    int j;

    /*
    for (i = 0, j = 0; i < max; j++) {
        u32 val = 0;
        int k;
        int shift = 0;

        if (clear)
            i += 4;
        else
            for (k = 0; k < 4 && i < max; i++, k++, shift += 8)
                val |= (i & 0x1f) << shift;

        write_register(val, &int_regs->channel_msi_vector[j],
                   XDMA_OFS_INT_CTRL +
                       ((unsigned long)&int_regs
                        ->channel_msi_vector[j] -
                    (unsigned long)int_regs));
        dbg_init("vector %d, 0x%x.\n", j, val);
    }
    */
}

static void irq_msix_channel_teardown(struct anlogic_dev *xdev)
{
    
    struct anlogic_dma_engine *engine;
    int j = 0;
    int i = 0;
  pr_info("xdev->msix_enabled: %d\n", xdev->msix_enabled);
    if (!xdev->msix_enabled)
        return;

    //prog_irq_msix_channel(xdev, 1);
     pr_info("xdev->h2c_channel_max: %d\n", xdev->h2c_channel_max);
	 pr_info("xdev->c2h_channel_max: %d\n", xdev->c2h_channel_max);

    engine = xdev->engine_h2c;
    for (i = 0; i < xdev->h2c_channel_max; i++, j++, engine++) {
		 pr_info("engine->msix_irq_line: %d\n", engine->msix_irq_line);
        if (!engine->msix_irq_line)
            break;
        dbg_sg("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
               engine);
        free_irq(engine->msix_irq_line, engine);
    }

    engine = xdev->engine_c2h;
    for (i = 0; i < xdev->c2h_channel_max; i++, j++, engine++) {
		pr_info("engine->msix_irq_line: %d\n", engine->msix_irq_line);
        if (!engine->msix_irq_line)
            break;
        dbg_sg("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
               engine);
        free_irq(engine->msix_irq_line, engine);
    }
    
}

static int irq_msix_channel_setup(struct anlogic_dev *xdev)
{
    int i;
    int j;
    int rv = 0;
    u32 vector;
  
    struct anlogic_dma_engine *engine;

    if (!xdev) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    if (!xdev->msix_enabled)
        return 0;

    j = xdev->h2c_channel_max;
    engine = xdev->engine_h2c;
	 pr_info(" xdev->h2c_channel_max %d .h2c : %d,\n",
                 xdev->h2c_channel_max, xdev->c2h_channel_max);
    for (i = 0; i < xdev->h2c_channel_max; i++, engine++) {
	//for(i=0;i<4; i++) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        vector = pci_irq_vector(xdev->pdev, i);
#else
        vector = xdev->entry[i].vector;
#endif
 pr_info(" vector%d  is: %d\n",
                 i, vector);
        rv = request_irq(vector, anlogic_channel_irq, 0, xdev->mod_name,
                 engine);
		
        if (rv) {
            pr_info("requesti irq#%d failed %d, engine %s.\n",
                vector, rv, engine->name);
            return rv;
        }
		
		if (rv) {
            pr_info("requesti irq#%d failed %d,\n",
                vector, rv);
            return rv;
        }
        pr_info(" irq#%d.\n",vector);
		
        engine->msix_irq_line = vector;
		 pr_info("engine->msix_irq_line:%d.\n",engine->msix_irq_line);
    }
    
    engine = xdev->engine_c2h;
    for (i = 0; i < xdev->c2h_channel_max; i++, j++, engine++) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        vector = pci_irq_vector(xdev->pdev, j);
#else
        vector = xdev->entry[j].vector;
#endif
        rv = request_irq(vector, anlogic_channel_irq, 0, xdev->mod_name,
                 engine);
        if (rv) {
            pr_info("requesti irq#%d failed %d, engine %s.\n",
                vector, rv, engine->name);
            return rv;
        }
        pr_info("engine %s, irq#%d.\n", engine->name, vector);
        engine->msix_irq_line = vector;
		pr_info("engine->msix_irq_line:%d.\n",engine->msix_irq_line);
		
    }
         
    return 0;
}

static void irq_msix_user_teardown(struct anlogic_dev *xdev)
{
    int i;
    int j;

    if (!xdev) {
        pr_err("Invalid xdev\n");
        return;
    }

    if (!xdev->msix_enabled)
        return;

    j = xdev->h2c_channel_max + xdev->c2h_channel_max;

    prog_irq_msix_user(xdev, 1);

    for (i = 0; i < xdev->user_max; i++, j++) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        u32 vector = pci_irq_vector(xdev->pdev, j);
#else
        u32 vector = xdev->entry[j].vector;
#endif
        dbg_init("user %d, releasing IRQ#%d\n", i, vector);
        free_irq(vector, &xdev->user_irq[i]);
    }
   
}


static int irq_msi_setup(struct anlogic_dev *xdev, struct pci_dev *pdev)
{
    int rv;

    xdev->irq_line = (int)pdev->irq;
    rv = request_irq(pdev->irq, anlogic_isr, 0, xdev->mod_name, xdev);//IRQF_TRIGGER_RISING
    if (rv)
        dbg_init("Couldn't use IRQ#%d, %d\n", pdev->irq, rv);
    else
        dbg_init("Using IRQ#%d with 0x%p\n", pdev->irq, xdev);

    return rv;
}



static int irq_msix_user_setup(struct anlogic_dev *xdev)
{
    int i;
    int j = xdev->h2c_channel_max + xdev->c2h_channel_max;
    int rv = 0;
#if 0
    /* vectors set in probe_scan_for_msi() */
    
    for (i = 0; i < xdev->user_max; i++, j++) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
        u32 vector = pci_irq_vector(xdev->pdev, j);
#else
        u32 vector = xdev->entry[j].vector;
#endif
        rv = request_irq(vector, anlogic_user_irq, 0, xdev->mod_name,
                 &xdev->user_irq[i]);
        if (rv) {
            pr_info("user %d couldn't use IRQ#%d, %d\n", i, vector,
                rv);
            break;
        }
        pr_info("%d-USR-%d, IRQ#%d with 0x%p\n", xdev->idx, i, vector,
            &xdev->user_irq[i]);
    }

    /* If any errors occur, free IRQs that were successfully requested */
    if (rv) {
        for (i--, j--; i >= 0; i--, j--) {
#if KERNEL_VERSION(4, 12, 0) <= LINUX_VERSION_CODE
            u32 vector = pci_irq_vector(xdev->pdev, j);
#else
            u32 vector = xdev->entry[j].vector;
#endif
            free_irq(vector, &xdev->user_irq[i]);
        }
    }
#endif
    return rv;
}


static int irq_legacy_setup(struct anlogic_dev *xdev, struct pci_dev *pdev)
{
    u32 w;
    u8 val;
    void *reg;
    int rv;
/*
    pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &val);
    dbg_init("Legacy Interrupt register value = %d\n", val);
    if (val > 1) {
        val--;
        w = (val << 24) | (val << 16) | (val << 8) | val;*/
        /* Program IRQ Block Channel vactor and IRQ Block User vector
         * with Legacy interrupt value
         */
         /*
        reg = xdev->bar[xdev->config_bar_idx] + 0x2080; // IRQ user
        write_register(w, reg, 0x2080);
        write_register(w, reg + 0x4, 0x2084);
        write_register(w, reg + 0x8, 0x2088);
        write_register(w, reg + 0xC, 0x208C);
        reg = xdev->bar[xdev->config_bar_idx] + 0x20A0; // IRQ Block
        write_register(w, reg, 0x20A0);
        write_register(w, reg + 0x4, 0x20A4);
        
    }*/
   
    xdev->irq_line = (int)pdev->irq;
    
    rv = request_irq(pdev->irq, anlogic_isr, IRQF_SHARED, xdev->mod_name,
             xdev);
    if (rv)
        dbg_init("Couldn't use IRQ#%d, %d\n", pdev->irq, rv);
    else
        dbg_init("Using IRQ#%d with 0x%p\n", pdev->irq, xdev);
    //enable_irq(pdev->irq);
    return 0;
}

static void irq_teardown(struct anlogic_dev *xdev)
{
    
    if (xdev->msix_enabled) {
        irq_msix_channel_teardown(xdev);
        //irq_msix_user_teardown(xdev);
    } else if (xdev->irq_line != -1) {
        dbg_init("Releasing IRQ#%d\n", xdev->irq_line);
        free_irq(xdev->irq_line, xdev);
    }
}

static int irq_setup(struct anlogic_dev *xdev, struct pci_dev *pdev)
{
    pci_keep_intx_enabled(pdev);

    if (xdev->msix_enabled) {
        int rv = irq_msix_channel_setup(xdev);

        if (rv)
            return rv;
       /* rv = irq_msix_user_setup(xdev);
        if (rv)
            return rv;
        prog_irq_msix_channel(xdev, 0);
        prog_irq_msix_user(xdev, 0);
       */
        return 0;
    } else if (xdev->msi_enabled)
        return irq_msi_setup(xdev, pdev);

    return irq_legacy_setup(xdev, pdev);
}

static irqreturn_t user_irq_service(int irq, struct anlogic_user_irq *user_irq)
{
	unsigned long flags;

	if (!user_irq) {
		pr_err("Invalid user_irq\n");
		return IRQ_NONE;
	}

	if (user_irq->handler)
		return user_irq->handler(user_irq->user_idx, user_irq->dev);

	spin_lock_irqsave(&(user_irq->events_lock), flags);
	if (!user_irq->events_irq) {
		user_irq->events_irq = 1;
		wake_up_interruptible(&(user_irq->events_wq));
	}
	spin_unlock_irqrestore(&(user_irq->events_lock), flags);

	return IRQ_HANDLED;
}
/*
 * anlogic_channel_irq() - Interrupt handler for channel interrupts in MSI-X mode
 *
 * @dev_id pointer to xdma_dev
 */
static irqreturn_t anlogic_channel_irq(int irq, void *dev_id)
{
    struct anlogic_dev *xdev;
	xdev = (struct anlogic_dev*)dev_id;
    //struct xdma_engine *engine;
    //struct interrupt_regs *irq_regs;
//    void __iomem *addr;
    dbg_irq("(irq=%d) <<<< INTERRUPT service ROUTINE\n", irq);
//	addr = xdev->bar[0] + 0x1050;
//	iowrite32(0x5a5a1234,addr);
//	iowrite32(0x11223344,addr+0x10);
   /*
	if (!dev_id) {
        pr_err("Invalid dev_id on irq line %d\n", irq);
        return IRQ_NONE;
    }
*/
#if 0
    engine = (struct xdma_engine *)dev_id;
    xdev = engine->xdev;

    if (!xdev) {
        WARN_ON(!xdev);
        dbg_irq("%s(irq=%d) xdev=%p ??\n", __func__, irq, xdev);
        return IRQ_NONE;
    }

    irq_regs = (struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
                         XDMA_OFS_INT_CTRL);

    /* Disable the interrupt for this engine */
    write_register(
        engine->interrupt_enable_mask_value,
        &engine->regs->interrupt_enable_mask_w1c,
        (unsigned long)(&engine->regs->interrupt_enable_mask_w1c) -
            (unsigned long)(&engine->regs));
    /* Dummy read to flush the above write */
    read_register(&irq_regs->channel_int_pending);
    /* Schedule the bottom half */
    schedule_work(&engine->work);
#endif
    /*
     * RTO - need to protect access here if multiple MSI-X are used for
     * user interrupts
     */
    xdev->irq_count++;
    return IRQ_HANDLED;
}


/*
 * anlogic_user_irq() - Interrupt handler for user interrupts in MSI-X mode
 *
 * @dev_id pointer to xdma_dev
 */
static irqreturn_t anlogic_user_irq(int irq, void *dev_id)
{
   struct anlogic_user_irq *user_irq;

    dbg_irq("(irq=%d) <<<< INTERRUPT SERVICE ROUTINE\n", irq);

    if (!dev_id) {
        pr_err("Invalid dev_id on irq line %d\n", irq);
        return IRQ_NONE;
    }
    user_irq = (struct anlogic_user_irq *)dev_id;

    return user_irq_service(irq, user_irq);
}
/*
 * anlogic_isr() - Interrupt handler for legacy and msi interrupt
 *
 * @dev_id pointer to anlogic_dev
 */
static irqreturn_t anlogic_isr(int irq, void *dev_id)
{
    u32 ch_irq;
    u32 user_irq;
    u32 mask;
    struct anlogic_dev *xdev;
	 xdev = (struct anlogic_dev *)dev_id;
//  struct interrupt_regs *irq_regs;
    printk(KERN_WARNING" Interrupt Handler Start ..");
    dbg_irq("(irq=%d, dev 0x%p) <<<< ISR.\n", irq, dev_id);
   // dbg_irq("(irq=%d, dev 0x%p) <<<< ISR.\n", irq, dev_id);
   /* if (!dev_id) {
        pr_err("Invalid dev_id on irq line %d\n", irq);
        return -IRQ_NONE;
    }
	
   

    if (!xdev) {
        WARN_ON(!xdev);
        dbg_irq("%s(irq=%d) xdev=%p ??\n", __func__, irq, xdev);
        return IRQ_NONE;
    }
	*/
/*
    irq_regs = (struct interrupt_regs *)(xdev->bar[xdev->config_bar_idx] +
                         XDMA_OFS_INT_CTRL);
                         */

    /* read channel interrupt requests */
    //ch_irq = read_register(&irq_regs->channel_int_request);
    //dbg_irq("ch_irq = 0x%08x\n", ch_irq);

    /*
     * disable all interrupts that fired; these are re-enabled individually
     * after the causing module has been fully serviced.
     */
     /*
    if (ch_irq)
        channel_interrupts_disable(xdev, ch_irq);
    */

    /* read user interrupts - this read also flushes the above write */
    //user_irq = read_register(&irq_regs->user_int_request);
    //dbg_irq("user_irq = 0x%08x\n", user_irq);
/*
    if (user_irq) {
        int user = 0;
        u32 mask = 1;
        int max = xdev->user_max;

        for (; user < max && user_irq; user++, mask <<= 1) {
            if (user_irq & mask) {
                user_irq &= ~mask;
                user_irq_service(irq, &xdev->user_irq[user]);
            }
        }
    }
*/
/*
    mask = ch_irq & xdev->mask_irq_h2c;
    if (mask) {
        int channel = 0;
        int max = xdev->h2c_channel_max;

        // iterate over H2C (PCIe read) 
        for (channel = 0; channel < max && mask; channel++) {
            struct xdma_engine *engine = &xdev->engine_h2c[channel];

            // engine present and its interrupt fired? 
            if ((engine->irq_bitmask & mask) &&
                (engine->magic == MAGIC_ENGINE)) {
                mask &= ~engine->irq_bitmask;
                dbg_tfr("schedule_work, %s.\n", engine->name);
                schedule_work(&engine->work);
            }
        }
    }

    mask = ch_irq & xdev->mask_irq_c2h;
    if (mask) {
        int channel = 0;
        int max = xdev->c2h_channel_max;

        // iterate over C2H (PCIe write) 
        for (channel = 0; channel < max && mask; channel++) {
            struct xdma_engine *engine = &xdev->engine_c2h[channel];

            // engine present and its interrupt fired? 
            if ((engine->irq_bitmask & mask) &&
                (engine->magic == MAGIC_ENGINE)) {
                mask &= ~engine->irq_bitmask;
                dbg_tfr("schedule_work, %s.\n", engine->name);
                schedule_work(&engine->work);
            }
        }
    }
*/
    xdev->irq_count++;
    //dbg_irq("xdev->irq_count = %d\n",xdev->irq_count);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 );
    return IRQ_HANDLED;
}



static inline void xdma_device_flag_clear(struct anlogic_dev *xdev, unsigned int f)
{
    unsigned long flags;

    spin_lock_irqsave(&xdev->lock, flags);
    xdev->flags &= ~f;
    spin_unlock_irqrestore(&xdev->lock, flags);
}

static inline void xdma_device_flag_set(struct anlogic_dev *xdev, unsigned int f)
{
    unsigned long flags;

    spin_lock_irqsave(&xdev->lock, flags);
    xdev->flags |= f;
    spin_unlock_irqrestore(&xdev->lock, flags);
}


static int set_dma_mask(struct pci_dev *pdev)
{
    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    dbg_init("sizeof(dma_addr_t) == %ld\n", sizeof(dma_addr_t));
    /* 64-bit addressing capability for XDMA? */
    if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(64))) {
        /* query for DMA transfer */
        /* @see Documentation/DMA-mapping.txt */
        dbg_init("pci_set_dma_mask()\n");
        /* use 64-bit DMA */
        dbg_init("Using a 64-bit DMA mask.\n");
        /* use 32-bit DMA for descriptors */
        dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
        /* use 64-bit DMA, 32-bit for consistent */
    } else if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))) {
        dbg_init("Could not set 64-bit DMA mask.\n");
        dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
        /* use 32-bit DMA */
        dbg_init("Using a 32-bit DMA mask.\n");
    } else {
        dbg_init("No suitable DMA possible.\n");
        return -EINVAL;
    }

    return 0;
}

/*
 * Unmap the BAR regions that had been mapped earlier using map_bars()
 */
static void unmap_bars(struct anlogic_dev *xdev, struct pci_dev *dev)
{
    int i;

    for (i = 0; i < XDMA_BAR_NUM; i++) {
        /* is this BAR mapped? */
        if (xdev->bar[i]) {
            /* unmap BAR */
            pci_iounmap(dev, xdev->bar[i]);
            /* mark as unmapped */
            xdev->bar[i] = NULL;
        }
    }
}

static int map_single_bar(struct anlogic_dev *xdev, struct pci_dev *dev, int idx)
{
    resource_size_t bar_start;
    resource_size_t bar_len;
    resource_size_t map_len;

    bar_start = pci_resource_start(dev, idx);
    bar_len = pci_resource_len(dev, idx);
    map_len = bar_len;

    xdev->bar[idx] = NULL;

    /* do not map BARs with length 0. Note that start MAY be 0! */
    if (!bar_len) {
        pr_info("BAR #%d is not present - skipping\n", idx);
        return 0;
    }

    /* BAR size exceeds maximum desired mapping? */
    if (bar_len > INT_MAX) {
        pr_info("Limit BAR %d mapping from %llu to %d bytes\n", idx,
            (u64)bar_len, INT_MAX);
        map_len = (resource_size_t)INT_MAX;
    }
    /*
     * map the full device memory or IO region into kernel virtual
     * address space
     */
    dbg_init("BAR%d: %llu bytes to be mapped.\n", idx, (u64)map_len);
    xdev->bar[idx] = pci_iomap(dev, idx, map_len);

    if (!xdev->bar[idx]) {
        pr_info("Could not map BAR %d.\n", idx);
        return -1;
    }

    pr_info("BAR%d at 0x%llx mapped at 0x%p, length=%llu(/%llu)\n", idx,
        (u64)bar_start, xdev->bar[idx], (u64)map_len, (u64)bar_len);

    return (int)map_len;
}

/* map_bars() -- map device regions into kernel virtual address space
 *
 * Map the device memory regions into kernel virtual address space after
 * verifying their sizes respect the minimum sizes needed
 */
static int map_bars(struct anlogic_dev *xdev, struct pci_dev *dev)
{
    int rv;

#ifdef XDMA_CONFIG_BAR_NUM
    rv = map_single_bar(xdev, dev, XDMA_CONFIG_BAR_NUM);
    if (rv <= 0) {
        pr_info("%s, map config bar %d failed, %d.\n",
            dev_name(&dev->dev), XDMA_CONFIG_BAR_NUM, rv);
        return -EINVAL;
    }

    if (is_config_bar(xdev, XDMA_CONFIG_BAR_NUM) == 0) {
        pr_info("%s, unable to identify config bar %d.\n",
            dev_name(&dev->dev), XDMA_CONFIG_BAR_NUM);
        return -EINVAL;
    }
    xdev->config_bar_idx = XDMA_CONFIG_BAR_NUM;

    return 0;
#else
    int i;
    int bar_id_list[XDMA_BAR_NUM];
    int bar_id_idx = 0;
    //int config_bar_pos = 0;

    /* iterate through all the BARs */
    for (i = 0; i < XDMA_BAR_NUM; i++) {
        int bar_len;

        bar_len = map_single_bar(xdev, dev, i);
        pr_err("BAR%d len:%d\n",i,bar_len);
        if (bar_len == 0) {
            continue;
        } else if (bar_len < 0) {
            rv = -EINVAL;
            goto fail;
        }

        /* Try to identify BAR as XDMA control BAR */
        xdev->config_bar_idx = 0;
        /*
        if ((bar_len >= XDMA_BAR_SIZE) && (xdev->config_bar_idx < 0)) {
            if (is_config_bar(xdev, i)) {
                xdev->config_bar_idx = i;
                config_bar_pos = bar_id_idx;
                pr_info("config bar %d, pos %d.\n",
                    xdev->config_bar_idx, config_bar_pos);
            }
        }
        */

        bar_id_list[bar_id_idx] = i;
        bar_id_idx++;
    }
    /* 本移植新增：给用户逻辑挑一个 BAR（非 config BAR、非 IO、已映射）*/
    for (i = 0; i < XDMA_BAR_NUM; i++) {
        if ((i == xdev->config_bar_idx) || !xdev->bar[i])
            continue;
        if (pci_resource_flags(dev, i) & IORESOURCE_IO)
            continue;
        xdev->user_bar_idx = i;
        pr_info("user bar %d, len 0x%llx.\n", i,
                (unsigned long long)pci_resource_len(dev, i));
        break;
    }

    /* The XDMA config BAR must always be present */
    /*
    if (xdev->config_bar_idx < 0) {
        pr_info("Failed to detect XDMA config BAR\n");
        rv = -EINVAL;
        goto fail;
    }
    */
    /*
    rv = identify_bars(xdev, bar_id_list, bar_id_idx, config_bar_pos);
    if (rv < 0) {
        pr_err("Failed to identify bars\n");
        return rv;
    }
    */

    /* successfully mapped all required BAR regions */
    return 0;

fail:
    /* unwind; unmap any BARs that we did map */
    unmap_bars(xdev, dev);
    return rv;

#endif
}


static int request_regions(struct anlogic_dev *xdev, struct pci_dev *pdev)
{
    int rv;

    if (!xdev) {
        pr_err("Invalid xdev\n");
        return -EINVAL;
    }

    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    dbg_init("pci_request_regions()\n");
    rv = pci_request_regions(pdev, xdev->mod_name);
    /* could not request all regions? */
    if (rv) {
        dbg_init("pci_request_regions() = %d, device in use?\n", rv);
        /* assume device is in use so do not disable it later */
        xdev->regions_in_use = 1;
    } else {
        xdev->got_regions = 1;
    }

    return rv;
}


static inline void anlogic_device_flag_set(struct anlogic_dev *xdev, unsigned int f)
{
    unsigned long flags;

    spin_lock_irqsave(&xdev->lock, flags);
    xdev->flags |= f;
    spin_unlock_irqrestore(&xdev->lock, flags);
}

static struct anlogic_dev *alloc_dev_instance(struct pci_dev *pdev)
{
    //int i;
    struct anlogic_dev *xdev;
    //struct xdma_engine *engine;

    if (!pdev) {
        pr_err("Invalid pdev\n");
        return NULL;
    }

    /* allocate zeroed device book keeping structure */
    xdev = kzalloc(sizeof(struct anlogic_dev), GFP_KERNEL);
    if (!xdev) {
        pr_info("OOM, xdma_dev.\n");
        return NULL;
    }
    spin_lock_init(&xdev->lock);

    xdev->magic = MAGIC_DEVICE;
    xdev->config_bar_idx = -1;
    xdev->user_bar_idx = -1;
    xdev->bypass_bar_idx = -1;
    xdev->irq_line = -1;

    /* create a driver to device reference */
    xdev->pdev = pdev;
    dbg_init("xdev = 0x%p\n", xdev);
    
    /*setup work queue for 
    /* Set up data user IRQ data structures */
    /*
    for (i = 0; i < 16; i++) {
        xdev->user_irq[i].xdev = xdev;
        spin_lock_init(&xdev->user_irq[i].events_lock);
        init_waitqueue_head(&xdev->user_irq[i].events_wq);
        xdev->user_irq[i].handler = NULL;
        xdev->user_irq[i].user_idx = i; //0 based 
    }
   */
/*
    engine = xdev->engine_h2c;
    for (i = 0; i < XDMA_CHANNEL_NUM_MAX; i++, engine++) {
        spin_lock_init(&engine->lock);
        mutex_init(&engine->desc_lock);
        INIT_LIST_HEAD(&engine->transfer_list);
#if KERNEL_VERSION(4, 6, 0) <= LINUX_VERSION_CODE
        init_swait_queue_head(&engine->shutdown_wq);
        init_swait_queue_head(&engine->xdma_perf_wq);
#else
        init_waitqueue_head(&engine->shutdown_wq);
        init_waitqueue_head(&engine->xdma_perf_wq);
#endif
    }

    engine = xdev->engine_c2h;
    for (i = 0; i < XDMA_CHANNEL_NUM_MAX; i++, engine++) {
        spin_lock_init(&engine->lock);
        mutex_init(&engine->desc_lock);
        INIT_LIST_HEAD(&engine->transfer_list);
#if KERNEL_VERSION(4, 6, 0) <= LINUX_VERSION_CODE
        init_swait_queue_head(&engine->shutdown_wq);
        init_swait_queue_head(&engine->xdma_perf_wq);
#else
        init_waitqueue_head(&engine->shutdown_wq);
        init_waitqueue_head(&engine->xdma_perf_wq);
#endif
    }
    */

    return xdev;
}



#if KERNEL_VERSION(3, 5, 0) <= LINUX_VERSION_CODE
static void pci_enable_capability(struct pci_dev *pdev, int cap)
{
    pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, cap);
}
#else
static void pci_enable_capability(struct pci_dev *pdev, int cap)
{
    u16 v;
    int pos;

    pos = pci_pcie_cap(pdev);
    if (pos > 0) {
        pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &v);
        v |= cap;
        pci_write_config_word(pdev, pos + PCI_EXP_DEVCTL, v);
    }
}
#endif

static int engine_init(struct anlogic_dma_engine *engine, struct anlogic_dev *xdev,
               int offset, enum dma_data_direction dir, int channel)
{
    int rv;
    u32 val = 0;   /* 原本读 identifier 的那行被注释掉了，显式置 0 = MM 模式 */

    dbg_init("channel %d, offset 0x%x, dir %d.\n", channel, offset, dir);

    /* set magic */
    engine->magic = MAGIC_ENGINE;

    engine->channel = channel;

    /* engine interrupt request bit */
    engine->irq_bitmask = (1 << XDMA_ENG_IRQ_NUM) - 1;
    engine->irq_bitmask <<= (xdev->engines_num * XDMA_ENG_IRQ_NUM);
    engine->bypass_offset = xdev->engines_num * BYPASS_MODE_SPACING;

    /* parent */
    engine->xdev = xdev;
    /* register address */
    engine->regs = (xdev->bar[xdev->config_bar_idx] + offset);
    engine->sgdma_regs = xdev->bar[xdev->config_bar_idx] + offset +
                 SGDMA_OFFSET_FROM_CHANNEL;
    ///val = read_register(&engine->regs->identifier);
    if (val & 0x8000U)
        engine->streaming = 1;

    /* remember SG DMA direction */
    engine->dir = dir;
    sprintf(engine->name, "%d-%s%d-%s", xdev->idx,
        (dir == DMA_TO_DEVICE) ? "H2C" : "C2H", channel,
        engine->streaming ? "ST" : "MM");

    dbg_init("engine %p name %s irq_bitmask=0x%08x\n", engine, engine->name,
         (int)engine->irq_bitmask);

    /* initialize the deferred work for transfer completion */
    //INIT_WORK(&engine->work, engine_service_work);

    if (dir == DMA_TO_DEVICE)
        xdev->mask_irq_h2c |= engine->irq_bitmask;
    else
        xdev->mask_irq_c2h |= engine->irq_bitmask;
    xdev->engines_num++;

#if 0
    rv = engine_alloc_resource(engine);
    if (rv)
        return rv;

    rv = engine_init_regs(engine);
    if (rv)
        return rv;

    if (poll_mode)
        xdma_thread_add_work(engine);
#endif
    return 0;
}

static int probe_for_engine(struct anlogic_dev *xdev, enum dma_data_direction dir,
                int channel)
{
    struct engine_regs *regs;
    int offset = channel * CHANNEL_SPACING;
    u32 engine_id;
    u32 engine_id_expected;
    u32 channel_id;
    struct anlogic_dma_engine *engine;
    int rv;

    /* register offset for the engine */
    /* read channels at 0x0000, write channels at 0x1000,
     * channels at 0x100 interval
     */
    if (dir == DMA_TO_DEVICE) {
        engine_id_expected = XDMA_ID_H2C;
        engine = &xdev->engine_h2c[channel];
    } else {
        offset += H2C_CHANNEL_OFFSET;
        engine_id_expected = XDMA_ID_C2H;
        engine = &xdev->engine_c2h[channel];
    }

    regs = xdev->bar[xdev->config_bar_idx] + offset;
    //engine_id = get_engine_id(regs);
    //channel_id = get_engine_channel_id(regs);
    /*
    if ((engine_id != engine_id_expected) || (channel_id != channel)) {
        dbg_init(
            "%s %d engine, reg off 0x%x, id mismatch 0x%x,0x%x,exp 0x%x,0x%x, SKIP.\n",
            dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel, offset,
            engine_id, channel_id, engine_id_expected,
            channel_id != channel);
        return -EINVAL;
    }
   */
	/*
    dbg_init("found AXI %s %d engine, reg. off 0x%x, id 0x%x,0x%x.\n",
         dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel, offset,
         engine_id, channel_id);
*/
    /* allocate and initialize engine */
    rv = engine_init(engine, xdev, offset, dir, channel);
   /* if (rv != 0) {
        pr_info("failed to create AXI %s %d engine.\n",
            dir == DMA_TO_DEVICE ? "H2C" : "C2H", channel);
        return rv;
    }
*/
    return 0;
}

static int engine_destroy(struct anlogic_dev *xdev, struct anlogic_dma_engine *engine)
{
	if (!xdev) {
		pr_err("Invalid xdev\n");
		return -EINVAL;
	}

	if (!engine) {
		pr_err("dma engine NULL\n");
		return -EINVAL;
	}

	dbg_sg("Shutting down engine %s%d", engine->name, engine->channel);

	/* Disable interrupts to stop processing new events during shutdown */
	/*
	write_register(0x0, &engine->regs->interrupt_enable_mask,
		       (unsigned long)(&engine->regs->interrupt_enable_mask) -
			       (unsigned long)(&engine->regs));

	if (enable_credit_mp && engine->streaming &&
	    engine->dir == DMA_FROM_DEVICE) {
		u32 reg_value = (0x1 << engine->channel) << 16;
		struct sgdma_common_regs *reg =
			(struct sgdma_common_regs
				 *)(xdev->bar[xdev->config_bar_idx] +
				    (0x6 * TARGET_SPACING));
		write_register(reg_value, &reg->credit_mode_enable_w1c, 0);
	}

	if (poll_mode)
		xdma_thread_remove_work(engine);*/

	/* Release memory use for descriptor writebacks */
	//engine_free_resource(engine);

	memset(engine, 0, sizeof(struct anlogic_dma_engine));
	/* Decrement the number of engines available */
	xdev->engines_num--;
	return 0;
}


static void remove_engines(struct anlogic_dev *xdev)
{
	struct anlogic_dma_engine *engine;
	int i;
	int rv;

	if (!xdev) {
		pr_err("Invalid xdev\n");
		return;
	}

	/* iterate over channels */
	for (i = 0; i < xdev->h2c_channel_max; i++) {
		engine = &xdev->engine_h2c[i];
		if (engine->magic == MAGIC_ENGINE) {
			dbg_sg("Remove %s, %d", engine->name, i);
			rv = engine_destroy(xdev, engine);
			if (rv < 0)
				pr_err("Failed to destroy H2C engine %d\n", i);
			dbg_sg("%s, %d removed", engine->name, i);
		}
	}

	for (i = 0; i < xdev->c2h_channel_max; i++) {
		engine = &xdev->engine_c2h[i];
		if (engine->magic == MAGIC_ENGINE) {
			dbg_sg("Remove %s, %d", engine->name, i);
			rv = engine_destroy(xdev, engine);
			if (rv < 0)
				pr_err("Failed to destroy C2H engine %d\n", i);
			dbg_sg("%s, %d removed", engine->name, i);
		}
	}
}

static int probe_engines(struct anlogic_dev *xdev)
{
    int i;
    int rv = 0;

    if (!xdev) {
        pr_err("Invalid xdev\n");
        return -EINVAL;
    }

    /* iterate over channels */
    for (i = 0; i < xdev->h2c_channel_max; i++) {
        rv = probe_for_engine(xdev, DMA_TO_DEVICE, i);
        if (rv)
            break;
    }
    xdev->h2c_channel_max = i;

    for (i = 0; i < xdev->c2h_channel_max; i++) {
        rv = probe_for_engine(xdev, DMA_FROM_DEVICE, i);
        if (rv)
            break;
    }
    xdev->c2h_channel_max = i;

    return 0;
}

void *anlogic_device_open(const char *mname, struct pci_dev *pdev,int *user_max,int *h2c_channel_max, int *c2h_channel_max)
{
    struct anlogic_dev *xdev = NULL;
    int rv = 0;

    pr_info("%s device %s, 0x%p.\n", mname, dev_name(&pdev->dev), pdev);
    
    /* allocate zeroed device book keeping structure */
    xdev = alloc_dev_instance(pdev);
    if (!xdev)
        return NULL;
    
    xdev->mod_name = mname;
    xdev->user_max = *user_max;
    xdev->h2c_channel_max = *h2c_channel_max;
    xdev->c2h_channel_max = *c2h_channel_max;
    
   //pr_info("xdev->h2c_channel_max%d.\n",xdev->h2c_channel_max);
   //pr_info("xdev->c2h_channel_max%d.\n", xdev->c2h_channel_max);
   

    xdma_device_flag_set(xdev, XDEV_FLAG_OFFLINE);

    if (xdev->user_max == 0 || xdev->user_max > MAX_USER_IRQ)
        xdev->user_max = MAX_USER_IRQ;
    if (xdev->h2c_channel_max == 0 || xdev->h2c_channel_max > XDMA_CHANNEL_NUM_MAX)
        xdev->h2c_channel_max = XDMA_CHANNEL_NUM_MAX;
    if (xdev->c2h_channel_max == 0 || xdev->c2h_channel_max > XDMA_CHANNEL_NUM_MAX)
        xdev->c2h_channel_max = XDMA_CHANNEL_NUM_MAX;
   
    rv = pci_enable_device(pdev);
    if (rv) {
        dbg_init("pci_enable_device() failed, %d.\n", rv);
        goto err_enable;
    }
    
    /* keep INTx enabled */
    pci_check_intr_pend(pdev);

    /* enable relaxed ordering */
    pci_enable_capability(pdev, PCI_EXP_DEVCTL_RELAX_EN);

    /* enable extended tag */
    pci_enable_capability(pdev, PCI_EXP_DEVCTL_EXT_TAG);
    
    /* force MRRS to be 512 */
    rv = pcie_set_readrq(pdev, 256);
    if (rv)
        pr_info("device %s, error set PCI_EXP_DEVCTL_READRQ: %d.\n",
            dev_name(&pdev->dev), rv);
            
    /* enable bus master capability */
    pci_set_master(pdev);
    
    rv = request_regions(xdev, pdev);
    if (rv)
        goto err_regions;
    
    rv = map_bars(xdev, pdev);
    if (rv)
        goto err_map;
    
    rv = set_dma_mask(pdev);
    if (rv)
        goto err_mask;
        
    /*
    check_nonzero_interrupt_status(xdev);
    channel_interrupts_disable(xdev, ~0);
    user_interrupts_disable(xdev, ~0);
    read_interrupts(xdev);
    */
  
    
   rv = probe_engines(xdev);
    if (rv)
        goto err_engines;

    
    rv = enable_msi_msix(xdev, pdev);
    if (rv < 0)
        goto err_enable_msix;
    
    rv = irq_setup(xdev, pdev);
    if (rv < 0)
        goto err_interrupts;
    /*
    if (!poll_mode)
        channel_interrupts_enable(xdev, ~0);
    */
    
    /* Flush writes */
    /*read_interrupts(xdev);*/
    
    //store real parameter in xpdev
    *user_max = xdev->user_max;
    *h2c_channel_max = xdev->h2c_channel_max;
    *c2h_channel_max = xdev->c2h_channel_max;
    
    xdma_device_flag_clear(xdev, XDEV_FLAG_OFFLINE);
   /* 
   gReadBuffer = pci_alloc_consistent(pdev, BUF_SIZE, &gReadHWAddr);
   if (NULL == gReadBuffer) {
      printk(KERN_CRIT"%s: Init: Unable to allocate gBuffer.\n",mname);
      return (-1);
   }
  */
   // Print Read buffer size and address to kernel log
  /*
   printk(KERN_INFO"%s: Read Buffer Allocation: %X->%X\n", mname, (unsigned int)gReadBuffer, (unsigned int)gReadHWAddr);
     // Allocate the write buffer with size BUF_SIZE and return the starting address
  gWriteBuffer = pci_alloc_consistent(pdev, BUF_SIZE, &gWriteHWAddr);
  if (NULL == gWriteBuffer) {
    printk(KERN_CRIT"%s: Init: Unable to allocate gBuffer.\n",mname);
    return (-1);
  }
  // Print Write buffer size and address to kernel log  
  printk(KERN_INFO"%s: Write Buffer Allocation: %X->%X\n", mname, (unsigned int)gWriteBuffer, (unsigned int)gWriteHWAddr);

  gBaseVirt = xdev->bar[0];
  if (!gBaseVirt) {
    printk(KERN_WARNING"%s: Init: Could not remap memory.\n", mname);
    return (-1);
  } 
    printk(KERN_INFO"%s: Write Buffer Allocation: %X\n", mname, (unsigned int)gBaseVirt);
	*/
      // Initializing card registers
  //XPCIe_InitCard();
    //XPCIe_WriteReg(2, gReadHWAddr );        // Write: Write DMA TLP Address register with starting address
    //XPCIe_WriteReg(7, gWriteHWAddr);         // Write: Read DMA TLP Address register with starting address.
    //printk(KERN_INFO"gWriteHWAddr : 0x%X\n",gWriteHWAddr);
    //printk(KERN_INFO"gReadHWAddr: 0x%x\n", gReadHWAddr);
    
    //pci_write_config_dword(pdev,0x5c,0x0);  //set msi mask register
    //pci_write_config_dword(pdev,0x60,0x0);  //set msi mask register
    
    return (void *)xdev;
err_interrupts:
    irq_teardown(xdev); 
err_enable_msix:
    disable_msi_msix(xdev, pdev);
    
err_engines:
    remove_engines(xdev);
    
err_mask:
    unmap_bars(xdev, pdev);
err_map:
    if (xdev->got_regions)
        pci_release_regions(pdev);
err_regions:
    if (!xdev->regions_in_use)
        pci_disable_device(pdev);
err_enable:
    //xdev_list_remove(xdev);
    kfree(xdev);
    return NULL;
}

EXPORT_SYMBOL_GPL(anlogic_device_open);

void anlogic_device_close(struct pci_dev *pdev, void *dev_hndl)
{
    struct anlogic_dev *xdev = (struct anlogic_dev *)dev_hndl;

    dbg_init("pdev 0x%p, xdev 0x%p.\n", pdev, dev_hndl);

    if (!dev_hndl)
        return;
/*
    if (debug_check_dev_hndl(__func__, pdev, dev_hndl) < 0)
        return;
    */

    dbg_sg("remove(dev = 0x%p) where pdev->dev.driver_data = 0x%p\n", pdev,
           xdev);
    if (xdev->pdev != pdev) {
        dbg_sg("pci_dev(0x%lx) != pdev(0x%lx)\n",
               (unsigned long)xdev->pdev, (unsigned long)pdev);
    }
/*
    channel_interrupts_disable(xdev, ~0);
    user_interrupts_disable(xdev, ~0);
    read_interrupts(xdev);
*/
	pr_info("engine->msix_irq_line: %d\n", (xdev->engine_h2c[0]).msix_irq_line);  
    
    
	pr_info("engine->msix_irq_line: %d\n", (xdev->engine_h2c[0]).msix_irq_line);  

    irq_teardown(xdev);
	pr_info("engine->msix_irq_line: %d\n", (xdev->engine_h2c[0]).msix_irq_line);  
	 dbg_init("before disable_msi_msix.\n");
	
    disable_msi_msix(xdev, pdev);
	 dbg_init("before unmap_bars.\n");
	 remove_engines(xdev);

    unmap_bars(xdev, pdev);

    //if (xdev->got_regions) 
    {
        dbg_init("pci_release_regions 0x%p.\n", pdev);
        pci_release_regions(pdev);
    }

    //if (!xdev->regions_in_use) 
    {
        dbg_init("pci_disable_device 0x%p.\n", pdev);
        pci_disable_device(pdev);
    }

    //xdev_list_remove(xdev);
	 dbg_init("before kfree.\n");
    kfree(xdev);
}
EXPORT_SYMBOL_GPL(anlogic_device_close);
