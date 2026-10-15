/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PH1P100 EP 控制节点最小用法示例：通过 /dev/ANLOGIC-PCI0_control 的
 * ANLOGIC_IOCW / ANLOGIC_IOCR 对某个 BAR 的某个偏移做单 dw 读或写。
 *
 * 用法：
 *   ./pci_bar <bar_id 0..5> <offset>            # 读
 *   ./pci_bar <bar_id 0..5> <offset> <wdata>    # 写
 *
 * BAR1/BAR3/BAR5 没有各自的映射（BAR1 是 I/O BAR，BAR3/BAR5 是 64 位 BAR 的高半），
 * 驱动里对应 hw->bar[] 为 NULL，这里直接拒掉（规约 15）。
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include "anlogic_pci_ioctl.h"

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
        __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while (0)

char pCtrlDev[] = "/dev/ANLOGIC-PCI0_control";

int reg_write(int fd, uint32_t bar_id, uint32_t offset, uint32_t wdata)
{
    struct anlogic_ioc_bar gBar;

    gBar.bar_id = bar_id;
    gBar.bar_type = 0;
    gBar.bar_offaddr = offset;
    gBar.wdata = wdata;
    gBar.rdata = 0;

    return ioctl(fd, ANLOGIC_IOCW, &gBar);
}

int reg_read(int fd, uint32_t bar_id, uint32_t offset, uint32_t *rdata)
{
    struct anlogic_ioc_bar gBar;
    int ret;

    gBar.bar_id = bar_id;
    gBar.bar_type = 0;
    gBar.bar_offaddr = offset;
    gBar.wdata = 0;
    gBar.rdata = 0;
    ret = ioctl(fd, ANLOGIC_IOCR, &gBar);

    *rdata = gBar.rdata;
    return ret;
}

int main(int argc, char **argv)
{
    uint32_t bar_id, offset, wrdata;
    unsigned int reg_value = 0;
    uint32_t mode = 0;
    int fd;

    if (argc < 3 || argc > 4) {
        printf("Usage: %s <bar_id 0..5> <offset> [wdata]\n", argv[0]);
        return -1;
    }

    bar_id = strtoul(argv[1], 0, 0);
    if (bar_id >= 6) {
        printf("Invalid bar ID, must be 0..5.\n");
        return -1;
    }

    /* BAR1/BAR3/BAR5 没有独立映射，驱动会在 ioctl 里返回 -EINVAL，这里先挡掉 */
    if (bar_id == 1 || bar_id == 3 || bar_id == 5) {
        printf("BAR%d is not independently mapped (no dedicated resource).\n",
               bar_id);
        return -1;
    }

    offset = strtoul(argv[2], 0, 0);
    if (offset % 4 != 0) {
        printf("Offset must be 4byte aligned.\n");
        return -1;
    }

    if (argc == 4) {
        wrdata = strtoul(argv[3], 0, 0);
        printf("Pci bar%d: address: 0x%x write data: 0x%x.\n",
               bar_id, offset, wrdata);
        mode = 1;
    } else {
        mode = 2;
        printf("Pci bar%d: address: 0x%x read.\n", bar_id, offset);
    }

    if ((fd = open(pCtrlDev, O_RDWR | O_SYNC)) == -1)
        FATAL;
    printf("character device %s opened.\n", pCtrlDev);

    if (mode == 1) {
        reg_write(fd, bar_id, offset, wrdata);
        printf("Write complete.\n");
    } else {
        reg_read(fd, bar_id, offset, &reg_value);
        printf("Read data: 0x%x complete.\n", reg_value);
    }

    close(fd);

    return 0;
}
