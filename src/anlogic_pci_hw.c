// SPDX-License-Identifier: GPL-2.0
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动：资源与寄存器访问层。
 *
 * 负责打开/关闭一个 EP：使能设备、申请区间、映射全部 BAR、设置 DMA 掩码、
 * 分配收帧缓冲（anlogic_pci_frame.c）、登记中断、枚举 DMA 通道。
 *
 * 本层不接触任何用户态可控的地址；用户态可控的 MMIO 全在 anlogic_pci_ctrl.c
 * 的 ioctl / mmap 入口统一逐项校验。
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "anlogic_pci.h"

/* 每个探测成功的实例拿一个全局唯一的编号（节点名与通道名前缀用它） */
static DEFINE_IDA(anlogic_pci_instance_ida);

/* 0 = MSI-X，1 = MSI（默认），2 = Legacy */
static unsigned int interrupt_mode = 1;
module_param(interrupt_mode, uint, 0644);
MODULE_PARM_DESC(interrupt_mode, "0 - MSI-X, 1 - MSI, 2 - Legacy");

static irqreturn_t anlogic_pci_channel_irq(int irq, void *dev_id);
static irqreturn_t anlogic_pci_isr(int irq, void *dev_id);

/*
 * MSI/MSI-X 能力检查，取自 linux/pci/msi.c 的 pci_msi_check_device 思路：
 * 设备自身没关 MSI、总线链上没关 MSI、且 capability 链里确实有该 capability。
 */
static int anlogic_pci_msi_msix_capable(struct pci_dev *dev, int type)
{
    struct pci_bus *bus;

    if (!dev || dev->no_msi)
        return 0;

    for (bus = dev->bus; bus; bus = bus->parent)
        if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
            return 0;

    return pci_find_capability(dev, type) ? 1 : 0;
}

static void anlogic_pci_check_intr_pend(struct pci_dev *pdev)
{
    u16 v;

    pci_read_config_word(pdev, PCI_STATUS, &v);
    if (v & PCI_STATUS_INTERRUPT) {
        pr_info("%s PCI STATUS Interrupt pending 0x%x.\n",
                dev_name(&pdev->dev), v);
        pci_write_config_word(pdev, PCI_STATUS, PCI_STATUS_INTERRUPT);
    }
}

static void anlogic_pci_keep_intx_enabled(struct pci_dev *pdev)
{
    /*
     * 硬件 bug 规避：MSI/MSI-X 不可用时回落到 legacy，但 legacy 使能位没人管；
     * 若被关掉就再也 ack 不了中断，一切卡死。这里强制清掉 INTX_DISABLE。
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

static void anlogic_pci_enable_capability(struct pci_dev *pdev, int cap)
{
    pcie_capability_set_word(pdev, PCI_EXP_DEVCTL, cap);
}

static void anlogic_pci_disable_msi_msix(struct anlogic_pci_hw *hw,
                                         struct pci_dev *pdev)
{
    if (hw->msix_enabled) {
        pci_disable_msix(pdev);
        hw->msix_enabled = 0;
    } else if (hw->msi_enabled) {
        pci_disable_msi(pdev);
        hw->msi_enabled = 0;
    }
}

static int anlogic_pci_enable_msi_msix(struct anlogic_pci_hw *hw,
                                       struct pci_dev *pdev)
{
    int rv = 0;

    if (!hw) {
        pr_err("Invalid hw\n");
        return -EINVAL;
    }
    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    if (!interrupt_mode && anlogic_pci_msi_msix_capable(pdev, PCI_CAP_ID_MSIX)) {
        int req_nvec = hw->h2c_channel_max + hw->c2h_channel_max;

        pr_info("Enabling MSI-X\n");
        rv = pci_alloc_irq_vectors(pdev, req_nvec, req_nvec, PCI_IRQ_MSIX);
        pr_info("PCI alloc irq: %d vector\n", rv);
        if (rv < 0) {
            /* 分配失败不置标志，明确回落到 legacy，不把负值往外抛 */
            pr_err("Couldn't enable MSI-X mode: %d, fall back to legacy\n",
                   rv);
            hw->msix_enabled = 0;
            rv = 0;
        } else {
            hw->msix_enabled = 1;
        }
    } else if (interrupt_mode == 1 &&
               anlogic_pci_msi_msix_capable(pdev, PCI_CAP_ID_MSI)) {
        pr_info("pci_enable_msi()\n");
        rv = pci_enable_msi(pdev);
        if (rv < 0) {
            pr_err("Couldn't enable MSI mode: %d, fall back to legacy\n", rv);
            hw->msi_enabled = 0;
            rv = 0;
        } else {
            hw->msi_enabled = 1;
        }
    } else {
        pr_info("MSI/MSI-X not detected - using legacy interrupts\n");
    }

    return rv;
}

/* MSI-X：每个通道一个向量 */
static int anlogic_pci_irq_msix_channel_setup(struct anlogic_pci_hw *hw)
{
    struct anlogic_pci_engine *engine;
    int i, j, rv = 0;
    u32 vector;

    if (!hw) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }
    if (!hw->msix_enabled)
        return 0;

    j = hw->h2c_channel_max;
    engine = hw->engine_h2c;
    for (i = 0; i < hw->h2c_channel_max; i++, engine++) {
        vector = pci_irq_vector(hw->pdev, i);
        rv = request_irq(vector, anlogic_pci_channel_irq, 0, hw->mod_name,
                         engine);
        if (rv) {
            pr_info("request irq#%d failed %d, engine %s.\n",
                    vector, rv, engine->name);
            return rv;
        }
        engine->msix_irq_line = vector;
        engine->irq_registered = 1;
        pr_info("engine %s, irq#%d.\n", engine->name, vector);
    }

    engine = hw->engine_c2h;
    for (i = 0; i < hw->c2h_channel_max; i++, j++, engine++) {
        vector = pci_irq_vector(hw->pdev, j);
        rv = request_irq(vector, anlogic_pci_channel_irq, 0, hw->mod_name,
                         engine);
        if (rv) {
            pr_info("request irq#%d failed %d, engine %s.\n",
                    vector, rv, engine->name);
            return rv;
        }
        engine->msix_irq_line = vector;
        engine->irq_registered = 1;
        pr_info("engine %s, irq#%d.\n", engine->name, vector);
    }

    return 0;
}

static void anlogic_pci_irq_msix_channel_teardown(struct anlogic_pci_hw *hw)
{
    struct anlogic_pci_engine *engine;
    int i;

    pr_info("hw->msix_enabled: %d\n", hw->msix_enabled);
    if (!hw->msix_enabled)
        return;

    engine = hw->engine_h2c;
    for (i = 0; i < hw->h2c_channel_max; i++, engine++) {
        if (!engine->irq_registered)
            continue;
        pr_debug("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
                 engine);
        free_irq(engine->msix_irq_line, engine);
        engine->irq_registered = 0;
    }

    engine = hw->engine_c2h;
    for (i = 0; i < hw->c2h_channel_max; i++, engine++) {
        if (!engine->irq_registered)
            continue;
        pr_debug("Release IRQ#%d for engine %p\n", engine->msix_irq_line,
                 engine);
        free_irq(engine->msix_irq_line, engine);
        engine->irq_registered = 0;
    }
}

static int anlogic_pci_irq_msi_setup(struct anlogic_pci_hw *hw,
                                     struct pci_dev *pdev)
{
    int rv;

    /* 申请成功之后才写 irq_line；失败保持 -1，teardown 也就不会去 free */
    rv = request_irq(pdev->irq, anlogic_pci_isr, 0, hw->mod_name, hw);
    if (rv) {
        pr_info("Couldn't use IRQ#%d, %d\n", pdev->irq, rv);
        return rv;
    }
    hw->irq_line = (int)pdev->irq;
    hw->irq_registered = 1;
    pr_info("Using IRQ#%d with 0x%p\n", pdev->irq, hw);

    return 0;
}

static int anlogic_pci_irq_legacy_setup(struct anlogic_pci_hw *hw,
                                        struct pci_dev *pdev)
{
    int rv;

    rv = request_irq(pdev->irq, anlogic_pci_isr, IRQF_SHARED, hw->mod_name, hw);
    if (rv) {
        pr_info("Couldn't use IRQ#%d, %d\n", pdev->irq, rv);
        return rv;
    }
    hw->irq_line = (int)pdev->irq;
    hw->irq_registered = 1;
    pr_info("Using IRQ#%d with 0x%p\n", pdev->irq, hw);

    return 0;
}

static void anlogic_pci_irq_teardown(struct anlogic_pci_hw *hw)
{
    if (hw->msix_enabled) {
        anlogic_pci_irq_msix_channel_teardown(hw);
    } else if (hw->irq_registered) {
        pr_info("Releasing IRQ#%d\n", hw->irq_line);
        free_irq(hw->irq_line, hw);
        hw->irq_registered = 0;
        hw->irq_line = -1;
    }
}

static int anlogic_pci_irq_setup(struct anlogic_pci_hw *hw, struct pci_dev *pdev)
{
    anlogic_pci_keep_intx_enabled(pdev);

    if (hw->msix_enabled)
        return anlogic_pci_irq_msix_channel_setup(hw);

    if (hw->msi_enabled)
        return anlogic_pci_irq_msi_setup(hw, pdev);

    return anlogic_pci_irq_legacy_setup(hw, pdev);
}

/*
 * MSI-X 通道中断处理：本 EP 没有 MSI-X 能力，这条路径实际不会走；
 * dev_id 是登记中断时传进来的 engine，由它取回 hw。
 * 保持“中断上下文里不做 MMIO”的约定。
 */
static irqreturn_t anlogic_pci_channel_irq(int irq, void *dev_id)
{
    struct anlogic_pci_engine *engine = (struct anlogic_pci_engine *)dev_id;

    pr_debug("(irq=%d) <<<< INTERRUPT service ROUTINE\n", irq);

    if (engine && engine->hw)
        atomic_inc(&engine->hw->irq_count);

    return IRQ_HANDLED;
}

/* legacy / MSI 的中断处理：只计数，不做 MMIO */
static irqreturn_t anlogic_pci_isr(int irq, void *dev_id)
{
    struct anlogic_pci_hw *hw = (struct anlogic_pci_hw *)dev_id;

    pr_debug("(irq=%d, dev 0x%p) <<<< ISR.\n", irq, dev_id);

    if (hw)
        atomic_inc(&hw->irq_count);

    return IRQ_HANDLED;
}

static int anlogic_pci_set_dma_mask(struct pci_dev *pdev)
{
    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(64))) {
        pr_info("Using a 64-bit DMA mask.\n");
        /* 描述符用 32 位一致映射 */
        dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
    } else if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))) {
        pr_info("Could not set 64-bit DMA mask.\n");
        dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
        pr_info("Using a 32-bit DMA mask.\n");
    } else {
        pr_info("No suitable DMA possible.\n");
        return -EINVAL;
    }

    return 0;
}

static void anlogic_pci_unmap_bars(struct anlogic_pci_hw *hw,
                                   struct pci_dev *dev)
{
    int i;

    for (i = 0; i < ANLOGIC_PCI_BAR_NUM; i++) {
        if (hw->bar[i]) {
            pci_iounmap(dev, hw->bar[i]);
            hw->bar[i] = NULL;
            hw->bar_map_len[i] = 0;
        }
    }
}

static int anlogic_pci_map_bar(struct anlogic_pci_hw *hw, struct pci_dev *dev,
                               int idx)
{
    resource_size_t bar_start;
    resource_size_t bar_len;
    resource_size_t map_len;

    bar_start = pci_resource_start(dev, idx);
    bar_len = pci_resource_len(dev, idx);
    map_len = bar_len;

    hw->bar[idx] = NULL;
    hw->bar_map_len[idx] = 0;

    /* 长度为 0 的 BAR 不映射（start 允许是 0） */
    if (!bar_len) {
        pr_info("BAR #%d is not present - skipping\n", idx);
        return 0;
    }

    if (bar_len > INT_MAX) {
        pr_info("Limit BAR %d mapping from %llu to %d bytes\n", idx,
                (u64)bar_len, INT_MAX);
        map_len = (resource_size_t)INT_MAX;
    }

    hw->bar[idx] = pci_iomap(dev, idx, map_len);
    if (!hw->bar[idx]) {
        pr_info("Could not map BAR %d.\n", idx);
        return -1;
    }
    /* 边界校验一律用这个实际映射长度，而不是 pci_resource_len() */
    hw->bar_map_len[idx] = map_len;

    pr_info("BAR%d at 0x%llx mapped at 0x%p, length=%llu(/%llu)\n", idx,
            (u64)bar_start, hw->bar[idx], (u64)map_len, (u64)bar_len);

    return (int)map_len;
}

/*
 * 映射 BAR0..BAR5。本 EP 的控制/寄存器窗固定在 BAR0；用户逻辑窗是第一个
 * 「非 config BAR、非 IO、已映射」的 mem BAR（本板实测为 BAR2）。
 */
static int anlogic_pci_map_bars(struct anlogic_pci_hw *hw, struct pci_dev *dev)
{
    int i;
    int rv = 0;

    for (i = 0; i < ANLOGIC_PCI_BAR_NUM; i++) {
        int bar_len;

        bar_len = anlogic_pci_map_bar(hw, dev, i);
        pr_info("BAR%d len:%d\n", i, bar_len);
        if (bar_len == 0)
            continue;
        if (bar_len < 0) {
            rv = -EINVAL;
            goto fail;
        }
        hw->config_bar_idx = 0;
    }

    for (i = 0; i < ANLOGIC_PCI_BAR_NUM; i++) {
        if (i == hw->config_bar_idx || !hw->bar[i])
            continue;
        if (pci_resource_flags(dev, i) & IORESOURCE_IO)
            continue;
        hw->user_bar_idx = i;
        pr_info("user bar %d, len 0x%llx.\n", i,
                (unsigned long long)pci_resource_len(dev, i));
        break;
    }

    return 0;

fail:
    anlogic_pci_unmap_bars(hw, dev);
    return rv;
}

static int anlogic_pci_request_regions(struct anlogic_pci_hw *hw,
                                       struct pci_dev *pdev)
{
    int rv;

    if (!hw) {
        pr_err("Invalid hw\n");
        return -EINVAL;
    }
    if (!pdev) {
        pr_err("Invalid pdev\n");
        return -EINVAL;
    }

    rv = pci_request_regions(pdev, hw->mod_name);
    if (rv) {
        pr_info("pci_request_regions() = %d, device in use?\n", rv);
        /* 认为设备已被占用，后面就不要去 disable 它 */
        hw->regions_in_use = 1;
    } else {
        hw->got_regions = 1;
    }

    return rv;
}

/* 通道登记：本 EP 没有可读的 XDMA 配置 BAR，通道按固定布局登记 */
static int anlogic_pci_engine_init(struct anlogic_pci_engine *engine,
                                   struct anlogic_pci_hw *hw,
                                   enum dma_data_direction dir, int channel)
{
    engine->magic = ANLOGIC_PCI_MAGIC_ENGINE;
    engine->channel = channel;
    engine->hw = hw;
    engine->dir = dir;
    /* 每个通道一个中断位 */
    engine->irq_bitmask = (1 << hw->engines_num);

    snprintf(engine->name, sizeof(engine->name), "%d-%s%d-MM", hw->idx,
             (dir == DMA_TO_DEVICE) ? "H2C" : "C2H", channel);

    pr_info("engine %p name %s irq_bitmask=0x%08x\n", engine, engine->name,
            (int)engine->irq_bitmask);

    hw->engines_num++;

    return 0;
}

static int anlogic_pci_probe_engine(struct anlogic_pci_hw *hw,
                                    enum dma_data_direction dir, int channel)
{
    struct anlogic_pci_engine *engine;

    if (dir == DMA_TO_DEVICE)
        engine = &hw->engine_h2c[channel];
    else
        engine = &hw->engine_c2h[channel];

    return anlogic_pci_engine_init(engine, hw, dir, channel);
}

static int anlogic_pci_probe_engines(struct anlogic_pci_hw *hw)
{
    int i, rv = 0;

    if (!hw) {
        pr_err("Invalid hw\n");
        return -EINVAL;
    }

    for (i = 0; i < hw->h2c_channel_max; i++)
        if (anlogic_pci_probe_engine(hw, DMA_TO_DEVICE, i))
            break;
    hw->h2c_channel_max = i;

    for (i = 0; i < hw->c2h_channel_max; i++)
        if (anlogic_pci_probe_engine(hw, DMA_FROM_DEVICE, i))
            break;
    hw->c2h_channel_max = i;

    return rv;
}

static int anlogic_pci_engine_destroy(struct anlogic_pci_hw *hw,
                                      struct anlogic_pci_engine *engine)
{
    if (!hw) {
        pr_err("Invalid hw\n");
        return -EINVAL;
    }
    if (!engine) {
        pr_err("dma engine NULL\n");
        return -EINVAL;
    }

    pr_debug("Shutting down engine %s%d\n", engine->name, engine->channel);

    memset(engine, 0, sizeof(*engine));
    hw->engines_num--;

    return 0;
}

static void anlogic_pci_remove_engines(struct anlogic_pci_hw *hw)
{
    struct anlogic_pci_engine *engine;
    int i;

    if (!hw) {
        pr_err("Invalid hw\n");
        return;
    }

    for (i = 0; i < hw->h2c_channel_max; i++) {
        engine = &hw->engine_h2c[i];
        if (engine->magic == ANLOGIC_PCI_MAGIC_ENGINE)
            anlogic_pci_engine_destroy(hw, engine);
    }

    for (i = 0; i < hw->c2h_channel_max; i++) {
        engine = &hw->engine_c2h[i];
        if (engine->magic == ANLOGIC_PCI_MAGIC_ENGINE)
            anlogic_pci_engine_destroy(hw, engine);
    }
}

void *anlogic_pci_hw_open(const char *mod_name, struct pci_dev *pdev,
                          int *user_max, int *h2c_channel_max,
                          int *c2h_channel_max)
{
    struct anlogic_pci_hw *hw;
    int rv;

    if (!pdev) {
        pr_err("Invalid pdev\n");
        return NULL;
    }

    pr_info("%s device %s, 0x%p.\n", mod_name, dev_name(&pdev->dev), pdev);

    hw = kzalloc(sizeof(*hw), GFP_KERNEL);
    if (!hw) {
        pr_err("OOM, anlogic_pci_hw.\n");
        return NULL;
    }

    hw->magic = ANLOGIC_PCI_MAGIC_DEVICE;
    hw->pdev = pdev;
    hw->mod_name = mod_name;
    hw->config_bar_idx = -1;
    hw->user_bar_idx = -1;
    hw->irq_line = -1;
    hw->user_max = *user_max;
    hw->h2c_channel_max = *h2c_channel_max;
    hw->c2h_channel_max = *c2h_channel_max;

    mutex_init(&hw->lock);
    atomic_set(&hw->removed, 0);
    atomic_set(&hw->refs, 0);
    atomic_set(&hw->irq_count, 0);
    init_waitqueue_head(&hw->refs_wait);

    /* 全局唯一实例号：同型 EP 的第二张也能拿到不同的节点/通道名 */
    hw->idx = ida_alloc(&anlogic_pci_instance_ida, GFP_KERNEL);
    if (hw->idx < 0) {
        pr_err("no free instance index, %d.\n", hw->idx);
        kfree(hw);
        return NULL;
    }

    if (hw->user_max == 0 || hw->user_max > ANLOGIC_PCI_MAX_USER_IRQ)
        hw->user_max = ANLOGIC_PCI_MAX_USER_IRQ;
    if (hw->h2c_channel_max == 0 ||
        hw->h2c_channel_max > ANLOGIC_PCI_MAX_CHANNELS)
        hw->h2c_channel_max = ANLOGIC_PCI_MAX_CHANNELS;
    if (hw->c2h_channel_max == 0 ||
        hw->c2h_channel_max > ANLOGIC_PCI_MAX_CHANNELS)
        hw->c2h_channel_max = ANLOGIC_PCI_MAX_CHANNELS;

    rv = pci_enable_device(pdev);
    if (rv) {
        pr_err("pci_enable_device() failed, %d.\n", rv);
        goto err_enable;
    }
    hw->enabled = 1;

    /* 保留 INTx */
    anlogic_pci_check_intr_pend(pdev);

    /* 打开 relaxed ordering 与 extended tag */
    anlogic_pci_enable_capability(pdev, PCI_EXP_DEVCTL_RELAX_EN);
    anlogic_pci_enable_capability(pdev, PCI_EXP_DEVCTL_EXT_TAG);

    rv = pcie_set_readrq(pdev, 256);
    if (rv)
        pr_info("device %s, error set PCI_EXP_DEVCTL_READRQ: %d.\n",
                dev_name(&pdev->dev), rv);

    pci_set_master(pdev);

    rv = anlogic_pci_request_regions(hw, pdev);
    if (rv)
        goto err_regions;

    rv = anlogic_pci_map_bars(hw, pdev);
    if (rv)
        goto err_map;

    rv = anlogic_pci_set_dma_mask(pdev);
    if (rv)
        goto err_mask;

    /* 收帧缓冲：必须在 DMA 掩码设好之后分配（FPGA 用它当 MWr 目标） */
    rv = anlogic_pci_frame_alloc(hw);
    if (rv)
        goto err_frame;

    rv = anlogic_pci_probe_engines(hw);
    if (rv)
        goto err_engines;

    rv = anlogic_pci_enable_msi_msix(hw, pdev);
    if (rv < 0)
        goto err_enable_msix;

    rv = anlogic_pci_irq_setup(hw, pdev);
    if (rv < 0)
        goto err_interrupts;

    /*
     * 相机链路 / 推帧的武装：下 bit 或重启之后 FPGA 里一切归零，这一段把以前靠
     * 人工补的初始化步骤（使能位、推帧/门铃地址、按需重触发传感器）做完。
     * 失败**不**让 probe 失败：老 FPGA 映像没有这组寄存器，硬失败会让驱动装不上；
     * 它自己会把原因打全，并把 armed 留在 0，用户态可用 ANLOGIC_IOCLINKSTATUS 查。
     */
    rv = anlogic_pci_link_arm(hw);
    if (rv < 0)
        pr_err("链路：探测时武装失败 %d，驱动继续加载（用 ioctl 看状态、重试）。\n",
               rv);

    /* 把实际生效的参数回传给上层 */
    *user_max = hw->user_max;
    *h2c_channel_max = hw->h2c_channel_max;
    *c2h_channel_max = hw->c2h_channel_max;

    return hw;

err_interrupts:
    anlogic_pci_irq_teardown(hw);
err_enable_msix:
    anlogic_pci_disable_msi_msix(hw, pdev);
err_engines:
    anlogic_pci_remove_engines(hw);
err_frame:
    anlogic_pci_frame_free(hw);
err_mask:
    anlogic_pci_unmap_bars(hw, pdev);
err_map:
    if (hw->got_regions)
        pci_release_regions(pdev);
err_regions:
    if (hw->enabled)
        pci_disable_device(pdev);
err_enable:
    ida_free(&anlogic_pci_instance_ida, hw->idx);
    kfree(hw);
    return NULL;
}

void anlogic_pci_hw_close(struct pci_dev *pdev, void *dev_hndl)
{
    struct anlogic_pci_hw *hw = (struct anlogic_pci_hw *)dev_hndl;

    if (!dev_hndl)
        return;

    pr_debug("pdev 0x%p, hw 0x%p.\n", pdev, dev_hndl);

    if (hw->pdev != pdev)
        pr_info("pci_dev mismatch: hw 0x%p, pdev 0x%p.\n", hw->pdev, pdev);

    anlogic_pci_irq_teardown(hw);
    anlogic_pci_disable_msi_msix(hw, pdev);
    anlogic_pci_remove_engines(hw);
    /* 先让 FPGA 别再往这块缓冲推帧，再释放（否则 MWr 会打进已经被复用的内存） */
    anlogic_pci_link_disarm(hw);
    anlogic_pci_frame_free(hw);
    anlogic_pci_unmap_bars(hw, pdev);

    pci_release_regions(pdev);
    pci_disable_device(pdev);

    ida_free(&anlogic_pci_instance_ida, hw->idx);
    kfree(hw);
}

/*
 * 生命周期与串行化（修「收帧节点 fd/映射与卸载脱钩」与「驱动全程无锁」）：
 *
 *   - open 一个节点时用 hw_get() 增引用；hw->removed 置后一律拒绝新的取用；
 *   - 等 fd 的 release（hw_put）减引用；
 *   - remove 调 hw_quiesce()：先置 removed，再拿一次 hw->lock 排空正在跑的
 *     ioctl/mmap（它们在入口看到 removed 会尽快返回），最后等引用归零才让上层
 *     释放 hw、BAR 映射与收帧缓冲。等帧 ioctl 最长睡 600 s，每轮都看 removed。
 *
 * 引用只用 fd 数，mmap 出来的 VMA 不另计数（这里踩过坑，写清楚）：VMA 是由一个
 * 已打开的 fd mmap 出来的，内核把它钉在 vm_file 上（mmap_region 里 get_file），
 * fd 的 release（本层的 hw_put）要等最后一个 VMA 也消失才会被调用 —— 所以
 * refs==0 本身就意味着「既没有开着的 fd、也没有活着的映射」。
 * 反过来「在 vm_ops->open/close 里加一减一」是错的：内核只在 fork/split 时替
 * **新** VMA 调 vm_ops->open（mm/mmap.c 的 dup_mmap、mm/vma.c 的 split），首次
 * mmap 出来的那个 VMA 只会拿到一次 vm_ops->close（remove_vma），而且 mmap 失败
 * 后的回退路径也走 remove_vma —— 两个方向都不配对。实测：一次 frame_grab 取帧
 * 就把 refs 减成了 -1，rmmod 卡在 hw_quiesce 里出不来。
 */
int anlogic_pci_hw_get(struct anlogic_pci_hw *hw)
{
    int rv = 0;

    if (!hw)
        return -EINVAL;

    mutex_lock(&hw->lock);
    if (atomic_read(&hw->removed))
        rv = -ENODEV;
    else
        atomic_inc(&hw->refs);
    mutex_unlock(&hw->lock);

    return rv;
}

void anlogic_pci_hw_put(struct anlogic_pci_hw *hw)
{
    if (!hw)
        return;

    /*
     * 计账严格配对：put 比 get 多一次，refs 就会变成负数，而 quiesce 等的是
     * 「引用归零」，负数再也回不到 0（见下面 quiesce 里为什么这里必须拦住）。
     * 这里当场告警并把这一笔多余的 put 丢掉：多出来的 put 只丢一个引用，不会把
     * 卸载路径钉死；告警栈能指出是哪条路径多减了。
     */
    if (WARN_ON_ONCE(atomic_read(&hw->refs) <= 0))
        return;

    if (atomic_dec_and_test(&hw->refs))
        wake_up_all(&hw->refs_wait);
}

void anlogic_pci_hw_quiesce(struct anlogic_pci_hw *hw)
{
    int refs;

    if (!hw)
        return;

    atomic_set(&hw->removed, 1);

    /* 等在飞的 ioctl/mmap 让出锁；它们看到 removed 后会尽快返回 */
    mutex_lock(&hw->lock);
    mutex_unlock(&hw->lock);

    refs = atomic_read(&hw->refs);
    if (refs > 0)
        pr_info("hw 0x%p: wait for %d user reference(s) before teardown.\n",
                hw, refs);

    /*
     * 这是卸载路径上唯一的等待点，也是上一版把 rmmod 卡死的地方：
     *   wait_event() 是**不可中断**的，等不到的判据会让 rmmod 永久停在 D 状态，
     *   连 SIGKILL 都杀不掉 —— 关机时 systemd 一直报「Waiting for process: rmmod」。
     * 判据写成 <= 0 而不是 == 0：正常路径上引用只会从 1 减到 0（上面 hw_put 已经
     * 拦住多减），两种写法等价；万一将来又有人把计数减成负数，这里也只会放行卸载，
     * 不会再等一个永远不为 0 的条件。
     * 引用归零 = 既没有开着的 fd、也没有活着的映射（VMA 钉着 vm_file，fd 的 release
     * 要等最后一个 VMA 消失才跑），所以此时的等待不会漏掉任何还在访问 hw 的用户。
     */
    wait_event(hw->refs_wait, atomic_read(&hw->refs) <= 0);
}
