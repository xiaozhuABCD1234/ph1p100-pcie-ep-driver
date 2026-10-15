/* ==========================================================================
 * frame_grab.c — 从驱动分配的收帧缓冲（DMA）里取一帧，写成 PGM
 *
 * 这是「收帧缓冲」这一层的最小用户态：只做三件事 ——
 *   1) ANLOGIC_IOCFRAMEINFO 问出缓冲的物理地址、大小、几何与序号/计数；
 *   2) ANLOGIC_IOCFRAMEWAIT 等一帧就绪（轮询序号，本 EP 没有 MSI，不走中断）；
 *   3) 把那一槽像素写成 P5 灰度 PGM。
 *
 * 缓冲布局与 FPGA 的握手约定（握手时序、槽头字段）写在 include/anlogic_pci_ioctl.h：
 * 槽区从缓冲起点开始，槽 n 在 phys_addr + stride*n（槽头后才是像素），mailbox
 * 描述符在槽区之后、由 ioctl 报的 mb_off 定位（整页，不与槽区重叠）。本工具只读：
 *   - 打开设备用 O_RDONLY；mmap 用 PROT_READ（驱动也会拒绝可写映射）；
 *   - 不碰任何 BAR 寄存器，所以 本工程既有的寄存器访问规约在这里
 *     没有适用对象（该规约管的是对 EP 的 MMIO，本层读写的是系统内存）。
 *
 * 空转行为（板上还没接 FPGA 帧源时必须正常返回）：mailbox.seq 恒 0，
 * ANLOGIC_IOCFRAMEWAIT 到时返回 ready=0，本工具打印「无新帧」并以退出码 4 结束，
 * 不写文件、不报错。
 *
 * 退出码
 *   0 成功（PGM 已写出）
 *   1 命令行用法错误
 *   2 设备不可用（open/flock/ioctl/mmap 失败、缓冲描述符不符）
 *   3 收帧缓冲还没就绪（描述符与驱动报的不一致）
 *   4 等超时 / 没有新帧（空转的正常结果）
 *   5 帧一致性不过（槽头序号对不上、帧号在搬运期间绕回、长度不合理）
 *   6 输出文件或内存错误
 *
 * 用法：./frame_grab [-d /dev/ANLOGIC-PCI0_frame] [-t 超时ms] [-n] [-o 输出.pgm] [--info]
 *       -o        输出 PGM 文件名（默认 dma_frame.pgm），也可以直接写成位置参数
 *       -n        只在出现比当前更新的帧时返回（默认：缓冲里已有帧就直接取最新一帧）
 *       --info    只打印缓冲的物理地址与几何，不取帧（空转自检用）
 * ========================================================================== */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "anlogic_pci_ioctl.h"

#define DEF_DEV        "/dev/ANLOGIC-PCI0_frame"
#define DEF_OUT        "dma_frame.pgm"
#define DEF_TIMEOUT_MS 3000u

/*
 * 与驱动、FPGA 共用同一份布局：字段偏移算错就会取到错数据，编译期先钉死
 * （驱动侧 anlogic_pci_frame.c 里有同样的 static_assert）。
 */
_Static_assert(sizeof(struct anlogic_frame_mailbox) == 64,
               "mailbox 必须是 64 字节（FPGA 偏移约定）");
_Static_assert(offsetof(struct anlogic_frame_mailbox, seq) == 4 &&
               offsetof(struct anlogic_frame_mailbox, count) == 8 &&
               offsetof(struct anlogic_frame_mailbox, stride) == 20,
               "mailbox 字段偏移与 FPGA 约定不符");
_Static_assert(sizeof(struct anlogic_frame_slot_hdr) == 16,
               "槽头必须是 16 字节（像素起始偏移）");
_Static_assert(offsetof(struct anlogic_frame_slot_hdr, len) == 4 &&
               offsetof(struct anlogic_frame_slot_hdr, flags) == 8,
               "槽头字段偏移与 FPGA 约定不符");

enum exit_code {
    EX_OK            = 0,
    EX_USAGE         = 1,
    EX_DEV           = 2,
    EX_NOT_READY     = 3,
    EX_NO_FRAME      = 4,
    EX_INCONSISTENT  = 5,
    EX_OUTPUT        = 6,
};

static void usage(const char *prog)
{
    fprintf(stderr,
        "用法：%s [-d 设备节点] [-t 超时ms] [-n] [-o 输出.pgm] [--info] [输出.pgm]\n"
        "  -d  收帧缓冲节点，默认 %s\n"
        "  -t  等帧超时（毫秒，1..600000），默认 %u\n"
        "  -n  只在出现更新的帧时返回（默认：缓冲里已有帧就直接取最新一帧）\n"
        "  -o  输出 PGM 文件名，默认 %s\n"
        "  --info 只打印缓冲物理地址/几何/序号，不取帧\n"
        "退出码：0 成功，2 设备不可用，3 描述符不符，4 无新帧，5 一致性不过，6 输出错误\n",
        prog, DEF_DEV, DEF_TIMEOUT_MS, DEF_OUT);
}

/* 读 mailbox 的一个字段（mmap 上直接读；volatile + 屏障，别让编译器合并/提前） */
static uint32_t mb_field(const volatile uint32_t *p)
{
    uint32_t v = *p;

    __sync_synchronize();
    return v;
}

static int frame_info(int fd, struct anlogic_ioc_frame *info)
{
    memset(info, 0, sizeof(*info));
    if (ioctl(fd, ANLOGIC_IOCFRAMEINFO, info) != 0) {
        fprintf(stderr, "!! ANLOGIC_IOCFRAMEINFO 失败：%s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/*
 * 描述符自检：驱动刚分配完就会把几何写进 mailbox（magic 最后写），
 * 这里既核对 ioctl 报的，也核对 mmap 里读到的首部，免得两边对不上还去算地址。
 */
static int descriptor_check(const struct anlogic_ioc_frame *info)
{
    if (!info->phys_addr || !info->size || !info->slots || !info->slot_bytes ||
        !info->stride || !info->hdr_bytes || !info->width || !info->height ||
        !info->mb_off) {
        fprintf(stderr, "!! 驱动报的收帧缓冲描述符不完整："
                        "phys=0x%llx size=%llu stride=%u slot=%u hdr=%u slots=%u %ux%u mb_off=%u\n",
                info->phys_addr, info->size, info->stride, info->slot_bytes,
                info->hdr_bytes, info->slots, info->width, info->height,
                info->mb_off);
        return -1;
    }
    if (info->stride < (uint64_t)info->hdr_bytes + info->slot_bytes) {
        fprintf(stderr, "!! stride %u < 槽头 %u + 一帧 %u：布局对不上\n",
                info->stride, info->hdr_bytes, info->slot_bytes);
        return -1;
    }
    /* 槽区 [0, stride*slots) 必须装得下所有槽，且不能压到 mailbox */
    if (info->size < (uint64_t)info->hdr_bytes +
                     (uint64_t)info->stride * info->slots) {
        fprintf(stderr, "!! 缓冲 %llu 字节装不下 %u 槽 × 步长 %u\n",
                info->size, info->slots, info->stride);
        return -1;
    }
    if (info->mb_off & 0xFFF) {
        fprintf(stderr, "!! mb_off %u 不是页对齐：mailbox 得独占一页\n",
                info->mb_off);
        return -1;
    }
    if ((uint64_t)info->mb_off < (uint64_t)info->stride * info->slots) {
        fprintf(stderr, "!! mb_off %u 落在槽区（%u 槽 × 步长 %u）里：会互相覆盖\n",
                info->mb_off, info->slots, info->stride);
        return -1;
    }
    if ((uint64_t)info->mb_off + sizeof(struct anlogic_frame_mailbox) >
        info->size) {
        fprintf(stderr, "!! mailbox 偏移 %u + 64 字节超出缓冲 %llu\n",
                info->mb_off, info->size);
        return -1;
    }
    if (info->slot_bytes < (uint64_t)info->width * info->height)
        fprintf(stderr, "!! 警告：一帧只有 %u 字节，小于 %ux%u 灰度需要的 %u\n",
                info->slot_bytes, info->width, info->height,
                info->width * info->height);
    return 0;
}

static void print_info(const struct anlogic_ioc_frame *info)
{
    printf("=== 收帧缓冲 ===\n");
    printf("  物理地址（FPGA 的 MWr 基址）: 0x%llx\n", info->phys_addr);
    printf("  缓冲大小                    : %llu 字节（%llu KiB，页对齐）\n",
           info->size, info->size / 1024);
    printf("  槽位                        : %u 槽，步长 %u 字节，槽头 %u 字节"
           "（槽 n 在 phys + %u×n）\n",
           info->slots, info->stride, info->hdr_bytes, info->stride);
    printf("  mailbox                     : 缓冲内偏移 %u（独占一页，"
           "FPGA 把 seq/count 写这里）\n", info->mb_off);
    printf("  每帧                        : %ux%u 灰度 = %u 字节\n",
           info->width, info->height, info->slot_bytes);
    printf("  序号/计数（FPGA 写）        : seq=%u count=%u%s\n",
           info->seq, info->count,
           info->seq ? "" : "（seq 还是 0：FPGA 还没往这块缓冲写过帧）");
}

int main(int argc, char **argv)
{
    const char *dev_path = DEF_DEV;
    const char *out_path = NULL;
    unsigned int timeout_ms = DEF_TIMEOUT_MS;
    int wait_new = 0;
    int want_info = 0;
    struct anlogic_ioc_frame info;
    struct anlogic_ioc_frame_wait wait;
    volatile uint8_t *buf = MAP_FAILED;
    uint8_t *img = NULL;
    size_t npix;
    uint32_t seq2;
    int fd = -1;
    int rc = EX_OK;
    FILE *fp;

    static const struct option longopts[] = {
        { "dev",     required_argument, NULL, 'd' },
        { "timeout", required_argument, NULL, 't' },
        { "new",     no_argument,       NULL, 'n' },
        { "output",  required_argument, NULL, 'o' },
        { "info",    no_argument,       NULL, 1   },
        { "help",    no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };

    for (;;) {
        int c = getopt_long(argc, argv, "d:t:no:h", longopts, NULL);
        if (c == -1)
            break;
        switch (c) {
        case 'd':
            dev_path = optarg;
            break;
        case 't': {
            char *end = NULL;
            unsigned long v = strtoul(optarg, &end, 0);
            if (!end || *end || v == 0 || v > ANLOGIC_FRAME_MAX_WAIT_MS) {
                fprintf(stderr, "!! --timeout 需要 1..%u 的毫秒数（收到 '%s'）\n",
                        ANLOGIC_FRAME_MAX_WAIT_MS, optarg);
                return EX_USAGE;
            }
            timeout_ms = (unsigned int)v;
            break;
        }
        case 'n':
            wait_new = 1;
            break;
        case 'o':
            out_path = optarg;
            break;
        case 1:
            want_info = 1;
            break;
        case 'h':
            usage(argv[0]);
            return EX_OK;
        default:
            usage(argv[0]);
            return EX_USAGE;
        }
    }
    for (int i = optind; i < argc; i++) {
        if (!out_path)
            out_path = argv[i];
        else {
            fprintf(stderr, "!! 多余的参数：'%s'\n", argv[i]);
            return EX_USAGE;
        }
    }
    if (!out_path)
        out_path = DEF_OUT;

    /* ---- 1. 打开节点（只读）并独占） ---- */
    fd = open(dev_path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "!! 打不开 %s：%s\n", dev_path, strerror(errno));
        fprintf(stderr, "   （需要 root；驱动已 insmod 且设备已枚举；"
                        "节点名形如 /dev/ANLOGIC-PCI0_frame）\n");
        return EX_DEV;
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {   /* 一次只允许一个进程 */
        fprintf(stderr, "!! %s 已被其它进程占用（flock 失败：%s）\n",
                dev_path, strerror(errno));
        close(fd);
        return EX_DEV;
    }

    /* ---- 2. 问缓冲物理地址 / 大小 / 序号 ---- */
    if (frame_info(fd, &info) != 0) {
        close(fd);
        return EX_DEV;
    }
    if (descriptor_check(&info) != 0) {
        close(fd);
        return EX_NOT_READY;
    }
    print_info(&info);

    if (want_info) {
        close(fd);
        return EX_OK;
    }

    /* ---- 3. 只读映射整块缓冲，并核对描述符首部 ---- */
    buf = mmap(NULL, info.size, PROT_READ, MAP_SHARED, fd, 0);
    if (buf == MAP_FAILED) {
        fprintf(stderr, "!! mmap %llu 字节失败：%s\n", info.size, strerror(errno));
        close(fd);
        return EX_DEV;
    }
    /* 定位 mailbox：偏移由驱动报的 mb_off 给出，不自己按常量猜 */
    const volatile struct anlogic_frame_mailbox *mb =
        (const volatile struct anlogic_frame_mailbox *)(buf + info.mb_off);
    if (mb_field(&mb->magic) != ANLOGIC_FRAME_MAGIC) {
        fprintf(stderr, "!! mailbox（偏移 %u）的 magic 不是 0x%08x：驱动还没把"
                        "描述符写好，先别取帧\n", info.mb_off, ANLOGIC_FRAME_MAGIC);
        munmap((void *)buf, info.size);
        close(fd);
        return EX_NOT_READY;
    }

    /* ---- 4. 等帧：默认只要缓冲里有帧就取最新那帧；-n 则等一帧更新的 ---- */
    memset(&wait, 0, sizeof(wait));
    wait.last_seq = wait_new ? info.seq : 0;
    wait.timeout_ms = timeout_ms;
    printf("=== 等帧（轮询序号，超时 %u ms，last_seq=%u）===\n",
           timeout_ms, wait.last_seq);
    if (ioctl(fd, ANLOGIC_IOCFRAMEWAIT, &wait) != 0) {
        if (errno == EINTR)
            fprintf(stderr, "!! 等帧被信号打断\n");
        else
            fprintf(stderr, "!! ANLOGIC_IOCFRAMEWAIT 失败：%s\n", strerror(errno));
        munmap((void *)buf, info.size);
        close(fd);
        return EX_DEV;
    }
    if (!wait.ready) {
        printf("无新帧：seq=%u（last_seq=%u），count=%u —— "
               "FPGA 还没把帧写进这块缓冲，本次不写文件\n",
               wait.seq, wait.last_seq, wait.count);
        munmap((void *)buf, info.size);
        close(fd);
        return EX_NO_FRAME;
    }

    /* ---- 5. 定位槽位并核对槽头（顺序：先看 seq，再看 len/flags） ---- */
    unsigned int slot = wait.slot;
    if (slot >= info.slots) {
        fprintf(stderr, "!! 驱动报的槽位 %u 越界（共 %u 槽）\n", slot, info.slots);
        munmap((void *)buf, info.size);
        close(fd);
        return EX_INCONSISTENT;
    }
    const volatile struct anlogic_frame_slot_hdr *hdr =
        (const volatile struct anlogic_frame_slot_hdr *)
        (buf + (size_t)slot * info.stride);
    uint32_t hdr_seq = mb_field(&hdr->seq);
    uint32_t hdr_len = mb_field(&hdr->len);
    uint32_t hdr_flags = mb_field(&hdr->flags);
    printf("  新帧：seq=%u（count=%u），槽位 %u，槽头 {seq=%u, len=%u, flags=0x%x}\n",
           wait.seq, wait.count, slot, hdr_seq, hdr_len, hdr_flags);

    rc = EX_OK;
    if (hdr_seq != wait.seq) {
        fprintf(stderr, "!! 槽头序号 %u 与 mailbox.seq %u 不一致（FPGA 还没按约定"
                        "先写槽头再改序号？）\n", hdr_seq, wait.seq);
        rc = EX_INCONSISTENT;
    }
    if (!(hdr_flags & ANLOGIC_FRAME_FLAG_COMPLETE)) {
        fprintf(stderr, "!! 槽头 flags=0x%x 没标 COMPLETE（bit0）：这一槽可能还在被写\n",
                hdr_flags);
        rc = EX_INCONSISTENT;
    }

    /* ---- 6. 搬像素（不整帧就补 0 并告警） ---- */
    npix = (size_t)info.width * info.height;
    img = malloc(npix);
    if (!img) {
        fprintf(stderr, "!! malloc %zu 字节失败\n", npix);
        munmap((void *)buf, info.size);
        close(fd);
        return EX_OUTPUT;
    }
    memset(img, 0, npix);
    size_t copy = hdr_len;
    if (copy > info.slot_bytes) {
        fprintf(stderr, "!! 槽头 len=%u 超过一帧 %u 字节，按一帧夹紧\n",
                hdr_len, info.slot_bytes);
        copy = info.slot_bytes;
        rc = EX_INCONSISTENT;
    }
    if (copy > npix) {
        fprintf(stderr, "!! 槽头 len=%u 大于 %ux%u 需要的 %zu 字节，按图大小夹紧\n",
                hdr_len, info.width, info.height, npix);
        copy = npix;
    }
    memcpy(img, (const void *)(buf + (size_t)slot * info.stride + info.hdr_bytes),
           copy);
    if (copy < npix) {
        fprintf(stderr, "!! 帧数据只有 %zu/%zu 字节，缺的用 0 补（图不完整）\n",
                copy, npix);
        rc = EX_INCONSISTENT;
    }

    /* ---- 7. 搬运后复查：只有环形缓冲绕回一圈才可能盖掉刚取的这一帧 ---- */
    if (frame_info(fd, &info) != 0) {
        free(img);
        munmap((void *)buf, info.size);
        close(fd);
        return EX_DEV;
    }
    seq2 = info.seq;
    if (seq2 != wait.seq) {
        unsigned int advanced = seq2 - wait.seq;
        printf("  搬运期间序号 %u → %u（新出 %u 帧，环上一共 %u 槽）%s\n",
               wait.seq, seq2, advanced, info.slots,
               advanced >= info.slots ? " ← 已经绕回，这一槽可能被覆盖过！"
                                      : "（还没绕回，本帧没被覆盖）");
        if (advanced >= info.slots)
            rc = EX_INCONSISTENT;
    }

    /* ---- 8. 写 P5 灰度 PGM ---- */
    fp = fopen(out_path, "wb");
    if (!fp) {
        fprintf(stderr, "!! 打不开输出文件 %s：%s\n", out_path, strerror(errno));
        free(img);
        munmap((void *)buf, info.size);
        close(fd);
        return EX_OUTPUT;
    }
    if (fprintf(fp, "P5\n%u %u\n255\n", info.width, info.height) < 0 ||
        fwrite(img, 1, npix, fp) != npix || fflush(fp) != 0 || fclose(fp) != 0) {
        fprintf(stderr, "!! 写 %s 失败：%s\n", out_path, strerror(errno));
        free(img);
        munmap((void *)buf, info.size);
        close(fd);
        return EX_OUTPUT;
    }

    printf("=== 结果 ===\n");
    printf("  输出文件  : %s（P5 灰度 %ux%u，%zu 字节像素）\n",
           out_path, info.width, info.height, npix);
    printf("  取自      : 物理 0x%llx + 槽 %u（偏移 %llu）+ 槽头 %u\n",
           info.phys_addr, slot, (unsigned long long)slot * info.stride,
           info.hdr_bytes);
    printf("  帧序号    : %u（累计 %u 帧）\n", wait.seq, wait.count);
    printf("  一致性    : %s\n",
           rc == EX_OK ? "OK（槽头序号与 mailbox 一致、flags 标了 COMPLETE、"
                         "搬运期间没绕回）"
                       : "**不通过**（图已写出，但请勿当可信帧用；原因见上面的告警）");

    free(img);
    munmap((void *)buf, info.size);
    close(fd);
    return rc;
}
