/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Anlogic PH1P100 PCIe EP 驱动与用户态程序共用的 ioctl 协议 —— 全仓库唯一一份。
 *
 * 这里的结构体布局、字段类型与 ANLOGIC_IOC* 编号是板上用户态程序的 ABI：
 * 只允许增补，不允许改动已有编号、字段类型与顺序。
 *
 * 访问规约（先验 MAGIC、单次 ≤4 dw、32 位对齐、全 F 熔断、ioctl 边界校验）见
 * ，驱动侧逐条实现在 anlogic_pci_ctrl.c。
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

/*
 * 收帧缓冲（DMA 一致性内存）：FPGA 侧用 MWr 把整帧推进来，用户态只读。
 *
 * 缓冲由驱动 dma_alloc_coherent 分配（物理地址连续、页对齐）。内存布局：
 *   偏移 0 起 stride*slots   槽区：stride*i 处是 struct anlogic_frame_slot_hdr
 *                           （16 字节），其后是一帧像素；i = (seq-1) % slots
 *   偏移 mb_off（页对齐）       struct anlogic_frame_mailbox（64 字节，独占一页）
 * FPGA 侧的 MWr 目标 = phys_addr + stride * 槽位号（0..slots-1），像素从槽头之后开始；
 * ioctl 把 mb_off 一并报给用户态，用户态靠它定位 mailbox（不要自己按固定偏移猜）。
 *
 * 握手约定（驱动只读、不写这些字段，只在分配时填描述符）：
 *   - 驱动分配完就把 magic/slot_bytes/slots/stride/width/height 写进 mailbox，
 *     FPGA 可以先读它拿几何参数；magic == ANLOGIC_FRAME_MAGIC 才算有效。
 *     magic 必须是最后一笔写，且前面用屏障挡开（写到 DRAM 必须可见才算就绪）。
 *   - 每写完一帧：先写该槽的槽头（seq/len/flags），**最后**才把 mailbox.seq
 *     更新成该帧序号（seq 从 1 起、每帧 +1，槽位 = (seq-1) % slots），同时
 *     mailbox.count 也 +1。序号那一笔 MWr 不能带 Relaxed Ordering 位，否则
 *     可能先于帧数据到达（PCIe 允许 posted write 重排）。
 *   - 主机侧一律轮询 mailbox.seq 判「帧就绪」，本 EP 目前只有 Legacy 中断、
 *     没有 MSI，所以这一层不做中断；FPGA 没接上时 seq 恒 0 →「无新帧」。
 *   - 槽区布局在驱动加载后固定不变：slot_bytes / slots / stride 不随帧变化，FPGA
 *     不要自行换算 stride，一律用 ioctl 报出来的 mb_off 与 stride。
 */
#define ANLOGIC_FRAME_MAGIC     0x4D524641u   /* 小端字节序为 'A','F','R','M' */

/* 收帧缓冲的规模上限：驱动侧拿它们夹 module param，等待超时的上限两边共用 */
#define ANLOGIC_FRAME_MAX_SLOTS         16
#define ANLOGIC_FRAME_MAX_SLOT_BYTES    (64u * 1024u * 1024u)
#define ANLOGIC_FRAME_MAX_WAIT_MS       600000u

struct anlogic_frame_mailbox {
    unsigned int magic;        /* 驱动写 ANLOGIC_FRAME_MAGIC：本描述符有效 */
    unsigned int seq;          /* FPGA 写：已完成帧的序号，每帧 +1，从 1 起；0 = 还没帧 */
    unsigned int count;        /* FPGA 写：累计完成帧数（单调 +1，用来分辨序号回绕） */
    unsigned int slot_bytes;   /* 一帧像素字节数 */
    unsigned int slots;        /* 槽位数 */
    unsigned int stride;       /* 槽位步长（字节，页对齐） */
    unsigned int width;        /* 每帧宽度（像素） */
    unsigned int height;       /* 每帧高度（像素） */
    unsigned int reserved[8];  /* 补齐到 64 字节 */
};

struct anlogic_frame_slot_hdr {
    unsigned int seq;          /* FPGA 写：本槽是第几帧，取帧时要与 mailbox.seq 核对 */
    unsigned int len;          /* FPGA 写：本槽有效像素字节数（≤ slot_bytes） */
    unsigned int flags;        /* FPGA 写：ANLOGIC_FRAME_FLAG_COMPLETE 表示整帧完整 */
    unsigned int reserved;
};

#define ANLOGIC_FRAME_FLAG_COMPLETE     0x1u

/* 收帧缓冲查询：FPGA 要拿 phys_addr 当 MWr 基址，用户态要拿几何与序号 */
struct anlogic_ioc_frame {
    unsigned long long  phys_addr;  /* 缓冲起始物理（PCIe 总线）地址 */
    unsigned long long  size;       /* 缓冲总字节数（页对齐） */
    unsigned int        stride;     /* 槽位步长 */
    unsigned int        slot_bytes; /* 一帧像素字节数 */
    unsigned int        hdr_bytes;  /* 槽头字节数（sizeof(anlogic_frame_slot_hdr)） */
    unsigned int        slots;      /* 槽位数 */
    unsigned int        width;      /* 每帧宽度（像素） */
    unsigned int        height;     /* 每帧高度（像素） */
    unsigned int        seq;        /* 当前序号（mailbox.seq；FPGA 没接上时 0） */
    unsigned int        count;      /* 累计完成帧数（mailbox.count） */
    unsigned int        mb_off;     /* mailbox 在缓冲内的字节偏移（页对齐，独占一页） */
};

/* 帧就绪轮询（不做中断）：last_seq 是用户已经消费到的序号，timeout_ms 用
 * ANLOGIC_FRAME_MAX_WAIT_MS 夹上限。 */
struct anlogic_ioc_frame_wait {
    unsigned int last_seq;    /* in  用户已消费到的序号（0 = 只要缓冲里有帧就返回） */
    unsigned int timeout_ms;  /* in  0 = 只查一次不等；>0 = 最多等这么久 */
    unsigned int seq;         /* out 当前序号 */
    unsigned int count;       /* out 当前累计帧数 */
    unsigned int ready;       /* out 1 = 有比 last_seq 新的帧（新帧在 (seq-1)%slots 槽） */
    unsigned int slot;        /* out 新帧所在槽位（ready=1 时有效） */
};

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
#define ANLOGIC_IOCFRAMEINFO    _IOR(ANLOGIC_IOC_MAGIC, 10, struct anlogic_ioc_frame)
#define ANLOGIC_IOCFRAMEWAIT    _IOWR(ANLOGIC_IOC_MAGIC, 11, struct anlogic_ioc_frame_wait)

#endif /* ANLOGIC_PCI_IOCTL_H */
