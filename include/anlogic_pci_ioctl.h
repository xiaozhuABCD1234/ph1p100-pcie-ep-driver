/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Anlogic PH1P100 PCIe EP 驱动与用户态程序共用的 ioctl 协议 —— 全仓库唯一一份。
 *
 * 这里的结构体布局、字段类型与 ANLOGIC_IOC* 编号是板上用户态程序的 ABI：
 * 只允许增补，不允许改动已有编号、字段类型与顺序。
 *
 * 访问规约（先验 MAGIC、单次 ≤4 dw、32 位对齐、全 F 熔断、ioctl 边界校验）见
 * doc/panic_risk_audit.md，驱动侧逐条实现在 anlogic_pci_ctrl.c。
 */
#ifndef ANLOGIC_PCI_IOCTL_H
#define ANLOGIC_PCI_IOCTL_H

#include <linux/ioctl.h>

/* 命令字类型，以及 ANLOGIC_IOCINFO 入参里的魔数 */
#define ANLOGIC_IOC_MAGIC   'x'
#define ANLOGIC_XCL_MAGIC   0X586C0C6C

enum ANLOGIC_IOC_TYPES {
    ANLOGIC_IOC_NOP,
    ANLOGIC_IOC_INFO,
    ANLOGIC_IOC_OFFLINE,
    ANLOGIC_IOC_ONLINE,
    ANLOGIC_IOC_MAX
};

struct anlogic_ioc_base {
    unsigned int magic;
    unsigned int command;
};

struct anlogic_ioc_info {
    struct anlogic_ioc_base base;
    unsigned short      vendor;
    unsigned short      device;
    unsigned short      subsystem_vendor;
    unsigned short      subsystem_device;
    unsigned int        dma_engine_version;
    unsigned int        driver_version;
    unsigned long long  feature_id;
    unsigned short      domain;
    unsigned char       bus;
    unsigned char       dev;
    unsigned char       func;
};

struct anlogic_ioc_bar {
    unsigned short    bar_id;
    unsigned short    bar_type;
    unsigned int      bar_offaddr;
    unsigned int      wdata;   /* write data */
    unsigned int      rdata;   /* read data */
};

/* BULK 单次读写最多搬运多少个 dw（驱动按此上限校验 bar_len） */
#define MAX_DATA_SIZE   100

struct anlogic_ioc_bar_bulk {
    unsigned short    bar_id;
    unsigned short    bar_type;
    unsigned int      bar_offaddr;
    unsigned int      bar_len;
    unsigned int      wdata[MAX_DATA_SIZE];   /* write data */
    unsigned int      rdata[MAX_DATA_SIZE];   /* read data */
};

/* 配置空间读写：寄存器偏移 + 值 */
typedef struct cfgwrite {
    int reg;
    int value;
} cfgwr;

/* IOCTL codes */
#define ANLOGIC_IOCINFO         _IOWR(ANLOGIC_IOC_MAGIC, ANLOGIC_IOC_INFO, struct anlogic_ioc_info)
#define ANLOGIC_IOCOFFLINE      _IO(ANLOGIC_IOC_MAGIC, ANLOGIC_IOC_OFFLINE)
#define ANLOGIC_IOCONLINE       _IO(ANLOGIC_IOC_MAGIC, ANLOGIC_IOC_ONLINE)
#define ANLOGIC_IOCW            _IOW(ANLOGIC_IOC_MAGIC, 4, struct anlogic_ioc_bar)
#define ANLOGIC_IOCR            _IOR(ANLOGIC_IOC_MAGIC, 5, struct anlogic_ioc_bar)
#define ANLOGIC_IOCBULKW        _IOW(ANLOGIC_IOC_MAGIC, 6, struct anlogic_ioc_bar_bulk)
#define ANLOGIC_IOCBULKR        _IOR(ANLOGIC_IOC_MAGIC, 7, struct anlogic_ioc_bar_bulk)
#define ANLOGIC_IOCCFGW         _IOW(ANLOGIC_IOC_MAGIC, 8, struct cfgwrite)
#define ANLOGIC_IOCCFGR         _IOR(ANLOGIC_IOC_MAGIC, 9, unsigned int)

#endif /* ANLOGIC_PCI_IOCTL_H */
