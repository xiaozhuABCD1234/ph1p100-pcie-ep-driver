/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Anlogic PH1P100 PCIe 端点（RK3588 主机侧）驱动内部头。
 *
 * 分三层：
 *   anlogic_pci_main.c   —— 模块与 PCI 驱动骨架（probe/remove/错误处理）
 *   anlogic_pci_hw.c     —— 资源与寄存器访问层（BAR、区间、DMA 掩码、中断、通道）
 *   anlogic_pci_chrdev.c / anlogic_pci_ctrl.c —— 字符设备与 ioctl 层
 *
 * 与用户态共用的 ioctl 协议只有 include/anlogic_pci_ioctl.h 一份。
 */
#ifndef ANLOGIC_PCI_H
#define ANLOGIC_PCI_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "anlogic_pci_ioctl.h"

/* PCI 驱动名，同时用作 request_irq / pci_request_regions 的 name */
#define ANLOGIC_PCI_DRV_NAME    "Anlogic-pcie"

/* 设备节点前缀：/dev/ANLOGIC-PCI0_control、/dev/ANLOGIC-PCI0_user */
#define ANLOGIC_PCI_NODE_NAME   "ANLOGIC-PCI"

/* 结构体自检魔数 */
#define ANLOGIC_PCI_MAGIC_DEVICE    0xDDDDDDDDUL
#define ANLOGIC_PCI_MAGIC_CHAR      0xCCCCCCCCUL
#define ANLOGIC_PCI_MAGIC_ENGINE    0xEEEEEEEEUL

#define ANLOGIC_PCI_BAR_NUM         6
#define ANLOGIC_PCI_MAX_CHANNELS    4
#define ANLOGIC_PCI_MAX_USER_IRQ    16

#define ANLOGIC_PCI_MINOR_BASE      0
#define ANLOGIC_PCI_MINOR_COUNT     255

/* mmap 用的 vma 标志：MMIO 页不可缓存、不可换出、不可 dump */
#define ANLOGIC_PCI_VMEM_FLAGS      (VM_IO | VM_DONTEXPAND | VM_DONTDUMP)

/* ANLOGIC_IOCINFO 上报的驱动版本号，沿用移植前的数值 */
#define ANLOGIC_PCI_DRV_VERSION_NUMBER  20200821

/* 控制节点与用户 BAR 窗口节点（两者共用同一套 file_operations） */
enum anlogic_pci_cdev_type {
    ANLOGIC_PCI_CDEV_USER = 0,  /* minor 0 -> /dev/ANLOGIC-PCI<n>_user */
    ANLOGIC_PCI_CDEV_CTRL = 1,  /* minor 1 -> /dev/ANLOGIC-PCI<n>_control */
    ANLOGIC_PCI_CDEV_TYPE_NUM
};

/* 已经创建出来的节点（用于销毁时判断） */
enum anlogic_pci_card_flags {
    ANLOGIC_PCI_CARD_CDEV_USER,
    ANLOGIC_PCI_CARD_CDEV_CTRL,
};

/* 一个 DMA 通道的登记信息（本 EP 没有 XDMA 配置 BAR，通道号是固定枚举出来的） */
struct anlogic_pci_engine {
    unsigned long magic;
    struct anlogic_pci_hw *hw;
    char name[16];
    enum dma_data_direction dir;
    int channel;
    u32 irq_bitmask;
    int msix_irq_line;
};

/* 资源与寄存器访问层：一个 EP 的 BAR 映射、区间、中断与通道登记 */
struct anlogic_pci_hw {
    unsigned long magic;
    struct pci_dev *pdev;
    int idx;

    const char *mod_name;

    void __iomem *bar[ANLOGIC_PCI_BAR_NUM];
    int config_bar_idx;
    int user_bar_idx;
    int regions_in_use;     /* probe 时设备已被别人占用 */
    int got_regions;        /* pci_request_regions() 成功 */

    int user_max;
    int h2c_channel_max;
    int c2h_channel_max;

    int irq_count;
    int irq_line;           /* -1 表示还没登记中断 */
    int msi_enabled;
    int msix_enabled;

    int engines_num;
    struct anlogic_pci_engine engine_h2c[ANLOGIC_PCI_MAX_CHANNELS];
    struct anlogic_pci_engine engine_c2h[ANLOGIC_PCI_MAX_CHANNELS];

    u64 feature_id;         /* ANLOGIC_IOCINFO 上报，本板恒 0 */
};

/* 一个字符设备实例（控制节点或用户 BAR 窗口） */
struct anlogic_pci_cdev {
    unsigned long magic;
    struct anlogic_pci_card *card;
    struct anlogic_pci_hw *hw;
    dev_t cdevno;
    struct cdev cdev;
    int bar;                /* 该节点对外暴露的 BAR（mmap/ioctl 用） */
    struct device *sys_device;
};

/* 一个探测成功实例的全部信息（PCI 驱动 data） */
struct anlogic_pci_card {
    unsigned long magic;
    struct pci_dev *pdev;
    struct anlogic_pci_hw *hw;
    int major;
    int user_max;
    int h2c_channel_max;
    int c2h_channel_max;
    unsigned int flags;
    struct anlogic_pci_cdev ctrl_cdev;
    struct anlogic_pci_cdev user_cdev;
};

/* anlogic_pci_hw.c */
void *anlogic_pci_hw_open(const char *mod_name, struct pci_dev *pdev,
                          int *user_max, int *h2c_channel_max,
                          int *c2h_channel_max);
void anlogic_pci_hw_close(struct pci_dev *pdev, void *hw);

/* anlogic_pci_chrdev.c */
int anlogic_pci_chrdev_init(void);
void anlogic_pci_chrdev_cleanup(void);
int anlogic_pci_card_create_interfaces(struct anlogic_pci_card *card);
void anlogic_pci_card_destroy_interfaces(struct anlogic_pci_card *card);

int anlogic_pci_cdev_check(const char *fname, struct anlogic_pci_cdev *cdev);
int anlogic_pci_cdev_open(struct inode *inode, struct file *file);
int anlogic_pci_cdev_close(struct inode *inode, struct file *file);

/* anlogic_pci_ctrl.c */
void anlogic_pci_ctrl_init(struct anlogic_pci_cdev *cdev);

#endif /* ANLOGIC_PCI_H */
