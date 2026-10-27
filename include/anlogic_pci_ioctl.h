/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Anlogic PH1P100 PCIe EP 驱动与用户态程序共用的 ioctl 协议 —— 全仓库唯一一份。
 *
 * 这里的结构体布局、字段类型与 ANLOGIC_IOC* 编号是板上用户态程序的 ABI：
 * 只允许增补，不允许改动已有编号、字段类型与顺序。
 *
 * 这些访问规约（先读 BAR0+0x40 验 MAGIC、单次不超过 4 个双字、32 位对齐、读到
 * 0xffffffff 立即熔断、ioctl 边界校验）由 anlogic_pci_ctrl.c 的 ioctl 路径逐条实现；
 * /dev/..._user 的 mmap 是只放 offset 0 一页的裸窗口，逐 offset 白名单与熔断由
 * 用户态自持（cam_grab 就是刻意直接 mmap sysfs resource0 的）。
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

/*
 * BULK 单次 ioctl 可以带多少个 dw（驱动按此上限校验 bar_len，再把整段按
 * 4 dw 一块推进；每个读回值都先过 BAR0 的 0x40 MAGIC 门禁，读到 0xffffffff
 * 立即以 -EIO 熔断）。
 */
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
 * FPGA 侧的 MWr 目标 = phys_addr + stride * 槽位号 + hdr_bytes（= 该槽**像素区起点**，
 * 槽头那 16 字节由驱动写，不能被推来的帧盖掉），像素从槽头之后开始；
 * 本板推帧引擎不逐帧轮换目标地址，所以推来的帧永远落在槽 0。
 * ioctl 把 mb_off 一并报给用户态，用户态靠它定位 mailbox（不要自己按固定偏移猜）。
 *
 * 握手约定：
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
 *
 * 本板推帧通路的实际分工（与上面那套约定的差异，按 FPGA 侧现状写实）：
 *   推帧引擎（fpga/rtl/ep_side/ddr/ddr_frame_pusher.v）只把整帧 MWr 到主机给的
 *   目标地址（0xB0/0xB4），**不写槽头、也不碰 mailbox.seq/count**；它只在推完
 *   一帧后往门铃地址（0xB8/0xBC）单独写一个 dw，内容是本帧的已推帧数（1 起）。
 *   所以那三项由驱动在看门铃时补齐：门铃写进 mailbox 页里的 push_dbell 字，
 *   驱动见到它变大就把对应槽的槽头、mailbox.seq/count 一并写好，用户态照旧
 *   只轮询 mailbox.seq。推帧目标地址是固定的一块（推帧引擎不做逐帧轮换），
 *   因此多槽下只有槽 0 会收到推来的帧 —— frame_slots 默认取 1。
 */
#define ANLOGIC_FRAME_MAGIC     0x4D524641u   /* 小端字节序为 'A','F','R','M' */

/*
 * T07：主机侧槽布局与 R10F 传输长度（三层长度分别命名，别再混用）。
 *   host 侧偏移 0..15      : struct anlogic_frame_slot_hdr（16 字节，**布局不变**）
 *   host 侧偏移 16..127    : 112 字节零 padding（驱动清零，不参与任何 CRC）
 *   host 侧偏移 128        : R10F 头（128 字节）
 *   之后                   : packed RAW10 有效载荷 raw_bytes = payload_bytes
 *   末尾                   : DMA 4KiB 对齐尾零（transport tail）
 * FPGA 的 MWr 目标 = 槽基址 + hdr_bytes(=128)；传输总量 = dma_bytes。
 */
#define ANLOGIC_FRAME_HDR_BYTES      128u        /* 传输数据起点（含 16B 槽头 + 112B 零 pad） */
#define ANLOGIC_FRAME_PREFIX_PAD     112u        /* 16 + 112 = 128 */
#define ANLOGIC_FRAME_R10F_HDR       128u        /* R10F 头长度（与 host 前缀是两层 128B） */
#define ANLOGIC_FRAME_DEF_WIDTH      1920u
#define ANLOGIC_FRAME_DEF_HEIGHT     1080u
#define ANLOGIC_FRAME_RAW_BYTES      2592000u    /* 1920x1080 packed RAW10（stride 2400B/行） */
#define ANLOGIC_FRAME_PAYLOAD_BYTES  ANLOGIC_FRAME_RAW_BYTES
#define ANLOGIC_FRAME_DMA_BYTES      2592768u    /* ceil((128+2592000)/4096)*4096 = 633*4096 */
#define ANLOGIC_FRAME_TAIL_PAD       (ANLOGIC_FRAME_DMA_BYTES - \
                                      (ANLOGIC_FRAME_R10F_HDR + ANLOGIC_FRAME_PAYLOAD_BYTES)) /* 640B */
#define ANLOGIC_FRAME_DEF_SLOT_BYTES ANLOGIC_FRAME_DMA_BYTES
#define ANLOGIC_FRAME_DEF_SLOTS      1u

/* 4KiB 对齐工具（stride/slot 计算都要求无溢出） */
#define ANLOGIC_FRAME_ALIGN4K(x)     (((x) + 4095u) & ~4095u)

/* 收帧缓冲的规模上限：驱动侧拿它们夹 module param，等待超时的上限两边共用 */
#define ANLOGIC_FRAME_MAX_SLOTS         16
#define ANLOGIC_FRAME_MAX_SLOT_BYTES    (64u * 1024u * 1024u)
#define ANLOGIC_FRAME_MAX_WAIT_MS       600000u

struct anlogic_frame_mailbox {
    unsigned int magic;        /* 驱动写 ANLOGIC_FRAME_MAGIC：本描述符有效 */
    unsigned int seq;          /* 已完成帧的序号，每帧 +1，从 1 起；0 = 还没帧 */
    unsigned int count;        /* 累计完成帧数（单调 +1，用来分辨序号回绕） */
    unsigned int slot_bytes;   /* 一帧像素字节数 */
    unsigned int slots;        /* 槽位数 */
    unsigned int stride;       /* 槽位步长（字节，页对齐） */
    unsigned int width;        /* 每帧宽度（像素） */
    unsigned int height;       /* 每帧高度（像素） */
    unsigned int push_dbell;   /* 推帧门铃：EP 写本帧的已推帧数（1 起），驱动见它变大就补 seq/count/槽头 */
    unsigned int reserved[7];  /* 补齐到 64 字节 */
};

struct anlogic_frame_slot_hdr {
    unsigned int seq;          /* 本槽是第几帧，取帧时要与 mailbox.seq 核对 */
    unsigned int len;          /* 本槽有效像素字节数（≤ slot_bytes） */
    unsigned int flags;        /* ANLOGIC_FRAME_FLAG_COMPLETE 表示整帧完整 */
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

/* BAR0 寄存器窗的 MAGIC 口（与驱动 anlogic_pci_ctrl.c、用户态 cam_grab 一致） */
#define ANLOGIC_BAR_MAGIC_OFF   0x40u
#define ANLOGIC_BAR_MAGIC_VAL   0x50434946u

/* 读取到全 F 即视为设备已不可用（PCIe 链路断开/复位），两侧都立即熔断 */
#define ANLOGIC_MMIO_ALL_F      0xffffffffu

/*
 * BAR0 寄存器窗里「相机链路 / 推帧」用到的偏移与位定义，表见
 * fpga/rtl/ep_side/img/ep_bar_win.v 头部那张寄存器表（驱动与用户态共用这一份，
 * 用户态解码状态字时按这里的位定义解，别在工具里另抄一套）。
 */
#define ANLOGIC_BAR_CTRL_OFF        0x44u   /* CTRL：bit 定义见下 */
#define ANLOGIC_BAR_FRAME_CNT_OFF   0x4Cu   /* 帧缓存已完成帧数 */
#define ANLOGIC_BAR_CAM_A_STAT_OFF  0x80u   /* A 路相机状态字（帧缓存写源就是 A 路） */
#define ANLOGIC_BAR_CAM_B_STAT_OFF  0x84u   /* B 路相机状态字 */
#define ANLOGIC_BAR_DDR_SEQ_OFF     0xA0u   /* 已整帧落 DDR3 的帧号 */
#define ANLOGIC_BAR_DDR_BASE_OFF    0xA4u   /* 该帧在 DDR3 里的字节基址 */
#define ANLOGIC_BAR_DDR_STATUS_OFF  0xA8u   /* DDR 握手状态 */
#define ANLOGIC_BAR_DDR_LEN_OFF     0xACu   /* 每帧字节数 */
#define ANLOGIC_BAR_PUSH_DST_LO_OFF 0xB0u   /* 推帧目标地址低 32 位 */
#define ANLOGIC_BAR_PUSH_DST_HI_OFF 0xB4u   /* 推帧目标地址高 32 位 */
#define ANLOGIC_BAR_PUSH_DB_LO_OFF  0xB8u   /* 门铃地址低 32 位 */
#define ANLOGIC_BAR_PUSH_DB_HI_OFF  0xBCu   /* 门铃地址高 32 位 */
#define ANLOGIC_BAR_PUSH_DONE_OFF   0xC0u   /* 已推帧数（1 起） */
#define ANLOGIC_BAR_PUSH_STATUS_OFF 0xC4u   /* 推帧状态 */
#define ANLOGIC_BAR_PUSH_ERR_OFF    0xC8u   /* 最近一次推帧错误码 */
#define ANLOGIC_BAR_PUSH_TLP_OFF    0xCCu   /* 最近一帧发出的 MWr 数（正常 2400） */

/* 本驱动要读写的寄存器里偏移最大的那个（+1），用来判 BAR0 映射长度够不够。
 * T07：RKA1 安全扩展把窗口推到 0x12C（0x100..0x11C 合同寄存器 + 0x120..0x12C
 * CSI 诊断），所以这里必须 >= 0x130。旧值 0xD0 会让扩展寄存器读写全部越界。 */
#define ANLOGIC_BAR_LINK_WIN_END    0x130u

/* ---- T07：RKA1 安全扩展窗口（contract/registers.json，0x100..0x11C）---- */
#define ANLOGIC_BAR_TX_CAP_OFF      0x100u   /* DMA_CAP_BYTES：主机分配容量（字节） */
#define ANLOGIC_BAR_TX_SESSION_OFF  0x104u   /* SESSION_ID：非零会话号 */
#define ANLOGIC_BAR_TX_CMD_OFF      0x108u   /* TX_CMD：W1，读恒 0 */
#define ANLOGIC_BAR_TX_ACK_SEQ_OFF  0x10Cu   /* ACK_SEQ：待匹配完成序号（门铃值） */
#define ANLOGIC_BAR_TX_ACK_SESS_OFF 0x110u   /* ACK_SESSION：待匹配会话号 */
#define ANLOGIC_BAR_TX_STATUS_OFF   0x114u   /* TX_STATUS：RO */
#define ANLOGIC_BAR_EXT_MAGIC_OFF   0x118u   /* EXT_MAGIC：RO，0x31414B52 = "RKA1" */
#define ANLOGIC_BAR_EXT_VERSION_OFF 0x11Cu   /* EXT_VERSION：RO，major<<16|minor */

#define ANLOGIC_RKA1_EXT_MAGIC      0x31414B52u  /* 小端字节序 = 'R','K','A','1' */
#define ANLOGIC_RKA1_EXT_VERSION    0x00010000u  /* major 1 / minor 0 */
#define ANLOGIC_RKA1_EXT_VERSION_MAJOR(v) (((v) >> 16) & 0xffffu)

/* TX_CMD（0x108）位：每笔写必须恰好一个 bit 置位 */
#define ANLOGIC_RKA1_CMD_ARM        0x1u
#define ANLOGIC_RKA1_CMD_ACK        0x2u
#define ANLOGIC_RKA1_CMD_CANCEL     0x4u

/* TX_STATUS（0x114）位 */
#define ANLOGIC_RKA1_ST_ARMED       0x01u
#define ANLOGIC_RKA1_ST_BUSY        0x02u
#define ANLOGIC_RKA1_ST_HELD        0x04u
#define ANLOGIC_RKA1_ST_REJECT      0x08u
#define ANLOGIC_RKA1_ST_DRAINING    0x10u

/* 推帧错误码（0xC8）在旧 0..4 之上新增 5..11（与 RTL ddr_frame_pusher.v 一致） */
#define ANLOGIC_PUSH_ERR_NOT_ARMED  5u   /* 有完整帧但没有 one-shot 额度 */
#define ANLOGIC_PUSH_ERR_ARM_REJ    6u   /* ARM 被拒（descriptor 校验失败） */
#define ANLOGIC_PUSH_ERR_CANCEL     7u   /* CANCEL 取消 */
#define ANLOGIC_PUSH_ERR_HELD       8u   /* HELD 期间源又出帧（忽略） */
#define ANLOGIC_PUSH_ERR_SRC        9u   /* 源/描述符不一致 */
#define ANLOGIC_PUSH_ERR_CFG        10u  /* 活动期写配置被拒 */
#define ANLOGIC_PUSH_ERR_BAD_ACK    11u  /* ACK 不匹配 */

/* CTRL（0x44）位定义 */
#define ANLOGIC_CTRL_ENABLE     0x00000001u  /* bit0  总使能 */
#define ANLOGIC_CTRL_TRIGGER    0x00000002u  /* bit1  单帧触发（自清，测试图源用） */
#define ANLOGIC_CTRL_MCLK_SEL   0x00000004u  /* bit2  电平一变就重跑一次传感器初始化（xclr 拉放 + 201 条表） */
#define ANLOGIC_CTRL_RSVD4      0x00000010u  /* bit4  保留：写 0/读 0（旧版当「帧缓存写源」用，T07 废弃） */
#define ANLOGIC_CTRL_DDR_EN     0x00000020u  /* bit5  每完成一帧自动搬进 DDR3 */
#define ANLOGIC_CTRL_PUSH_EN    0x00000040u  /* bit6  推帧模式使能（**不授权发包**，授权靠 RKA1 ARM） */
#define ANLOGIC_CTRL_PUSH_SRC   0x00000080u  /* bit7  推帧数据源：0 = DDR3，1 = 片内 ERAM */
#define ANLOGIC_CTRL_CAM_B      0x00000100u  /* bit8  推帧取哪一路（0 = A，1 = B） */

/*
 * 探测/初始化链路时使用的位：总使能 + DDR 搬运 + 推帧模式。
 * ★ T07 变更：
 *   · 不再包含 bit4（旧「相机写源」位，RTL 里是保留位，写 1 读回 0 ⇒ 旧 ARM 掩码
 *     0x71 与读回必然不符，是无效旧行为）；
 *   · bit6 只是「模式使能」，**不产生任何 DMA**：真正的发包授权是 RKA1 的
 *     ARM_ONE_SHOT（0x108.bit0），一次授权最多完成一帧，完成后 held 直到匹配 ACK；
 *   · bit7 保持 0（DDR3 源；ERAM 源会与 DDR 搬运抢帧缓存唯一读口）；
 *   · B 路把 ANLOGIC_CTRL_CAM_B 一并置上（掩码 0x161）。
 */
#define ANLOGIC_CTRL_ARM_BITS   (ANLOGIC_CTRL_ENABLE | ANLOGIC_CTRL_DDR_EN | \
                                 ANLOGIC_CTRL_PUSH_EN)
#define ANLOGIC_CTRL_ARM_BITS_B (ANLOGIC_CTRL_ARM_BITS | ANLOGIC_CTRL_CAM_B)

/* A 路相机状态字（0x80）里的 lane_valid：bit[4:1]，非 0 = MIPI 链路已锁 */
#define ANLOGIC_CAM_STAT_LANE_SHIFT     1u
#define ANLOGIC_CAM_STAT_LANE_MASK      0x0000001eu
#define ANLOGIC_CAM_STAT_BCLK_RUNNING   0x00000001u
#define ANLOGIC_CAM_STAT_CFG_DONE       0x00000020u
#define ANLOGIC_CAM_STAT_BUS_ERR        0x00000040u

#define ANLOGIC_CAM_LANE_VALID(stat) \
    (((stat) & ANLOGIC_CAM_STAT_LANE_MASK) >> ANLOGIC_CAM_STAT_LANE_SHIFT)

/* DDR 握手状态（0xA8）位定义 */
#define ANLOGIC_DDR_STATUS_VALID    0x00000001u
#define ANLOGIC_DDR_STATUS_BUSY     0x00000002u
#define ANLOGIC_DDR_STATUS_ERR      0x00000004u
#define ANLOGIC_DDR_STATUS_ALIVE    0x00000008u
#define ANLOGIC_DDR_STATUS_CAL_DONE 0x00000010u

/* 推帧状态（0xC4）位定义；[31:16] 是累计错误数 */
#define ANLOGIC_PUSH_STATUS_BUSY        0x00000001u
#define ANLOGIC_PUSH_STATUS_LAST_OK     0x00000002u
#define ANLOGIC_PUSH_STATUS_DB_SENT     0x00000004u
#define ANLOGIC_PUSH_STATUS_DDR_ALIVE   0x00000008u
#define ANLOGIC_PUSH_STATUS_DDR_CAL     0x00000010u

/* 推帧错误码（0xC8） */
#define ANLOGIC_PUSH_ERR_NONE       0u  /* 正常：数据与门铃都发了 */
#define ANLOGIC_PUSH_ERR_NO_DST     1u  /* 目标地址未配置 */
#define ANLOGIC_PUSH_ERR_WAIT_TO    2u  /* 等 tl_tx_wait 超时 */
#define ANLOGIC_PUSH_ERR_AXI        3u  /* AXI 读响应非 OKAY */
#define ANLOGIC_PUSH_ERR_NO_DB      4u  /* 门铃地址未配置（数据推了、门铃没发） */

/*
 * 相机链路 / 推帧的状态查询（ANLOGIC_IOCLINKSTATUS，只读）。
 * 字段全部来自 BAR0 寄存器（先过 0x40 MAGIC 门禁、读到全 F 立即熔断）与探测时
 * 记下的初始化结果；任何一项读失败就整个 ioctl 返回 -EIO，不做「部分成功」。
 */
struct anlogic_ioc_link_status {
    unsigned int ctrl;          /* 0x44 读回 */
    unsigned int frame_cnt;     /* 0x4C 帧缓存已完成帧数 */
    unsigned int cam_stat;      /* 0x80 A 路状态字原始值 */
    unsigned int lane_valid;    /* cam_stat[4:1] */
    unsigned int cam_locked;    /* 1 = lane_valid != 0（MIPI 链路已锁） */
    unsigned int ddr_seq;       /* 0xA0 */
    unsigned int ddr_base;      /* 0xA4 */
    unsigned int ddr_status;    /* 0xA8 原始值 */
    unsigned int ddr_ready;     /* 1 = alive && cal_done（DDR3 可用） */
    unsigned int ddr_len;       /* 0xAC 每次搬进 DDR3 的字节数 */
    unsigned int pushed;        /* 0xC0 已推帧数（1 起） */
    unsigned int push_status;   /* 0xC4 原始值 */
    unsigned int push_err;      /* 0xC8 最近一次推帧错误码 */
    unsigned int push_tlp_cnt;  /* 0xCC 最近一帧发出的 MWr 数（正常 2400） */
    unsigned int push_en;       /* 1 = CTRL bit6 已置位（推帧使能） */
    unsigned int push_src;      /* CTRL bit7（0 = 从 DDR3 推，1 = 从片内 ERAM 推） */
    unsigned int armed;         /* 1 = 推帧目标/门铃地址与使能位都已由驱动写好 */
    unsigned int dst_lo;        /* 0xB0 推帧目标地址低 32 位 */
    unsigned int dst_hi;        /* 0xB4 推帧目标地址高 32 位 */
    unsigned int db_lo;         /* 0xB8 门铃地址低 32 位 */
    unsigned int db_hi;         /* 0xBC 门铃地址高 32 位 */
    unsigned int seq;           /* mailbox.seq（驱动补齐后的已完成帧号） */
    unsigned int count;         /* mailbox.count */
};

/*
 * 手动重新触发一次相机初始化（ANLOGIC_IOCLINKRETRIG，链路掉了之后恢复用）。
 * 语义 = 把链路初始化用的 CTRL 位重新置上、并把 bit2 翻转一次（电平一变，顶层就重走
 * xclr 拉低 → 放开 → 重写整张 201 条寄存器表，见 ph1p_pcie_ep_top.v 的相机段）。
 * wait_ms = 0 表示只触发不等结果；> 0 表示最多等这么久，等「四路 lane 全对齐」
 * 或「帧计数开始动」（两者满足一个就算起来了，实测 lane 全部对齐要几秒，中间可能
 * 掉回 0，只看 lane_valid 非 0 会把正在恢复的状态误判成已锁定）。
 * frame_cnt / frames_flowing 是等待结束时 0x4C 的读数与它相对开始等时是否变过。
 */
struct anlogic_ioc_link_retrig {
    unsigned int wait_ms;         /* in  触发后最多等多久（0 = 不等，上限 ANLOGIC_LINK_MAX_WAIT_MS） */
    unsigned int ctrl;            /* out 触发后 0x44 读回 */
    unsigned int cam_stat;        /* out 触发后 0x80 读回 */
    unsigned int lane_valid;      /* out lane_valid（等完之后的实际值） */
    unsigned int cam_locked;      /* out 1 = lane_valid != 0 */
    unsigned int frame_cnt;       /* out 等完之后的 0x4C 帧计数 */
    unsigned int frames_flowing;  /* out 1 = 帧计数相对开始等时变过（帧已经在出） */
    unsigned int waited_ms;       /* out 实际等了多久 */
};

#define ANLOGIC_LINK_MAX_WAIT_MS    5000u

/* lane_valid 四条 lane 全对齐的取值 */
#define ANLOGIC_CAM_LANE_ALL        0xfu

/* 重设推帧目标/门铃地址时至少给一个（flags 位） */
#define ANLOGIC_PUSH_CFG_SET_DST    0x1u
#define ANLOGIC_PUSH_CFG_SET_DB     0x2u

/*
 * 复核 / 重设推帧目标与门铃地址（ANLOGIC_IOCPUSHCFG）。
 * flags = 0 只复核（out 字段填当前寄存器读回值）；置了 SET_ 位才写对应地址，
 * 写完必读回复核，读回与写入不一致时返回 -EIO。
 */
struct anlogic_ioc_push_cfg {
    unsigned int flags;         /* in  ANLOGIC_PUSH_CFG_SET_* 的组合，0 = 只复核 */
    unsigned int dst_lo;        /* in/out 推帧目标地址低 32 位（SET_DST 时按 in 用） */
    unsigned int dst_hi;        /* in/out 推帧目标地址高 32 位 */
    unsigned int db_lo;         /* in/out 门铃地址低 32 位（SET_DB 时按 in 用） */
    unsigned int db_hi;         /* in/out 门铃地址高 32 位 */
    unsigned int push_en;       /* out CTRL bit6 */
    unsigned int push_src;      /* out CTRL bit7 */
    unsigned int armed;         /* out 1 = 驱动已初始化 */
    unsigned int applied;       /* out 本次真正写进去的相关位 */
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
#define ANLOGIC_IOCLINKSTATUS   _IOR(ANLOGIC_IOC_MAGIC, 12, struct anlogic_ioc_link_status)
#define ANLOGIC_IOCLINKRETRIG   _IOWR(ANLOGIC_IOC_MAGIC, 13, struct anlogic_ioc_link_retrig)
#define ANLOGIC_IOCPUSHCFG      _IOWR(ANLOGIC_IOC_MAGIC, 14, struct anlogic_ioc_push_cfg)
/* T07 新增：RKA1 安全扩展操作（新结构体，不动旧结构体布局/编号） */
#define ANLOGIC_IOCRKA1         _IOWR(ANLOGIC_IOC_MAGIC, 15, struct anlogic_ioc_rka1)

/*
 * RKA1 安全扩展操作（T07 新增）。所有返回值都先过 EXT_MAGIC/EXT_VERSION 门禁：
 * 旧 peer FPGA 在 0x118 读回 DEAD_BEEF ⇒ probe 失败，本 ioctl 一律 -EOPNOTSUPP，
 * 绝不退回旧「写 CTRL 就连续推帧」的行为。
 */
enum ANLOGIC_RKA1_OPS {
    ANLOGIC_RKA1_OP_PROBE = 0,   /* 只探测：读 EXT_MAGIC/VERSION/TX_STATUS */
    ANLOGIC_RKA1_OP_CONFIGURE,   /* 写 CAP/SESSION（活动期会被 RTL 拒绝） */
    ANLOGIC_RKA1_OP_ARM,         /* 一键 ARM_ONE_SHOT（一个额度最多一帧） */
    ANLOGIC_RKA1_OP_ACK,         /* ACK_RELEASE：必须匹配 held(session,seq) */
    ANLOGIC_RKA1_OP_CANCEL       /* CANCEL：完成已开包、不发成功门铃、排空 */
};

struct anlogic_ioc_rka1 {
    unsigned int op;            /* in  ANLOGIC_RKA1_OP_* */
    unsigned int cap_bytes;     /* in  OP_CONFIGURE：主机分配容量（>= 帧 dma_bytes） */
    unsigned int session;       /* in  OP_CONFIGURE：非零会话号 */
    unsigned int ack_seq;       /* in  OP_ACK：门铃里收到的完成序号 */
    unsigned int ack_session;   /* in  OP_ACK：对应的会话号 */
    unsigned int ext_magic;     /* out 0x118 读回（必须 == ANLOGIC_RKA1_EXT_MAGIC） */
    unsigned int ext_version;   /* out 0x11C 读回 */
    unsigned int tx_status;     /* out 0x114 读回 */
    unsigned int push_err;      /* out 0xC8 最近错误码 */
    unsigned int push_status;   /* out 0xC4 推帧状态 */
    unsigned int push_done;     /* out 0xC0 已推帧数 */
    unsigned int armed;         /* out 1 = 本次 ARM 成功（TX_STATUS.bit0） */
    unsigned int held;          /* out 1 = TX_STATUS.bit2（等匹配 ACK） */
    unsigned int busy;          /* out 1 = TX_STATUS.bit1 */
    unsigned int rejected;      /* out 1 = TX_STATUS.bit3（粘滞拒绝/错误位） */
};

#endif /* ANLOGIC_PCI_IOCTL_H */
