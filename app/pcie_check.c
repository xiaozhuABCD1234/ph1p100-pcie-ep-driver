/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PH1P100 EP（RK3588 主机侧）链路与落位自检 —— 取代原来的 tools/opi_pcie_check.sh。
 *
 * 只读 sysfs，不需要 root、不需要 lspci、不碰设备寄存器，也不要求驱动已加载：
 *   1. EP 有没有被枚举出来（按 vendor:device = 1edb:abcd 认）
 *   2. 当前/最大链路速率与宽度
 *   3. BAR 资源表（对照期望：BAR0/2/4 = 1 MB mem、BAR1 = 256 B I/O）
 *   4. 驱动是否加载、/dev 节点是否在
 *
 * 用法：
 *   ./pcie_check [bdf]      bdf 形如 0000:01:00.0，缺省则扫描全部 PCI 设备
 * 退出码：0 找到 EP；1 没找到。
 *
 * 想看内核枚举/AER 日志请另行执行：
 *   dmesg | grep -iE '1edb|pcie.*link|aer'
 */

#include <dirent.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>

#define EP_VENDOR_ID   0x1edb
#define EP_DEVICE_ID   0xabcd

#define DEVPATH_MAX    256

static int read_hex_file(const char *dir, const char *name, unsigned *out)
{
    char path[DEVPATH_MAX];
    FILE *f;

    if (snprintf(path, sizeof path, "/sys/bus/pci/devices/%s/%s",
                 dir, name) >= (int)sizeof path)
        return -1;
    f = fopen(path, "r");
    if (!f)
        return -1;
    if (fscanf(f, "%x", out) != 1) {
        fclose(f);
        return -1;
    }
    fclose(f);
    return 0;
}

/* 读一行字符串（去掉结尾换行），读不到返回 -1 */
static int read_str_file(const char *dir, const char *name, char *buf, size_t len)
{
    char path[DEVPATH_MAX];
    FILE *f;
    size_t n;

    if (snprintf(path, sizeof path, "/sys/bus/pci/devices/%s/%s",
                 dir, name) >= (int)sizeof path)
        return -1;
    f = fopen(path, "r");
    if (!f)
        return -1;
    if (!fgets(buf, (int)len, f)) {
        fclose(f);
        return -1;
    }
    fclose(f);
    n = strlen(buf);
    while (n && (buf[n - 1] == '\n' || buf[n - 1] == ' '))
        buf[--n] = '\0';
    return 0;
}

static void show_bars(const char *bdf)
{
    char path[DEVPATH_MAX];
    FILE *f;
    unsigned long long start, end, flags;
    int i = 0;

    if (snprintf(path, sizeof path, "/sys/bus/pci/devices/%s/resource",
                 bdf) >= (int)sizeof path)
        return;
    f = fopen(path, "r");
    if (!f) {
        printf("    (读不到 resource)\n");
        return;
    }
    while (i < 6 && fscanf(f, "%llx %llx %llx", &start, &end, &flags) == 3) {
        if (start == 0 && end == 0 && flags == 0) {
            printf("    BAR%d  (未使用)\n", i);
        } else {
            const char *kind = (flags & 0x100ULL) ? "I/O" : "mem";

            printf("    BAR%d  %-3s 0x%010llx-0x%010llx  %llu bytes\n",
                   i, kind, start, end, end - start + 1);
        }
        i++;
    }
    fclose(f);
}

static int check_one(const char *bdf)
{
    char buf[128];
    unsigned vid = 0, did = 0, width = 0, max_width = 0;
    char speed[64] = "";
    char driver[256] = "";
    char link[DEVPATH_MAX];
    ssize_t n;

    printf("\n  BDF %s\n", bdf);

    int is_ep;

    if (read_hex_file(bdf, "vendor", &vid) ||
        read_hex_file(bdf, "device", &did)) {
        printf("    (读不到 vendor/device)\n");
        return -1;
    }
    is_ep = (vid == EP_VENDOR_ID && did == EP_DEVICE_ID);
    printf("    PCI ID: %04x:%04x  %s\n", vid, did,
           is_ep ? "(PH1P100 EP ✓)" : "(不是 PH1P100 EP)");

    if (snprintf(link, sizeof link, "/sys/bus/pci/devices/%s/driver", bdf) <
        (int)sizeof link) {
        n = readlink(link, driver, sizeof driver - 1);
        if (n > 0) {
            char *slash;

            driver[n] = '\0';
            slash = strrchr(driver, '/');
            printf("    驱动: %s\n", slash ? slash + 1 : driver);
        } else {
            printf("    驱动: （未绑定）\n");
        }
    }

    if (!read_str_file(bdf, "max_link_speed", speed, sizeof speed) &&
        !read_hex_file(bdf, "max_link_width", &max_width))
        printf("    LnkCap: Speed %s, Width x%u\n", speed, max_width);
    if (!read_str_file(bdf, "current_link_speed", speed, sizeof speed) &&
        !read_hex_file(bdf, "current_link_width", &width))
        printf("    LnkSta: Speed %s, Width x%u   (期望 5.0 GT/s PCIe, x2)\n",
               speed, width);

    printf("    BAR 资源:\n");
    show_bars(bdf);

    if (read_str_file(bdf, "modalias", buf, sizeof buf) == 0)
        printf("    modalias: %s\n", buf);

    return is_ep ? 0 : -1;
}

int main(int argc, char **argv)
{
    DIR *d;
    struct dirent *de;
    int found = 0;
    const char *want = NULL;

    if (argc > 1) {
        if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
            printf("Usage: %s [bdf]\n", argv[0]);
            return 0;
        }
        want = argv[1];
    }

    printf("== PH1P100 EP 链路自检（只读 sysfs，不需要 root）==\n");

    if (want) {
        found = (check_one(want) == 0);
    } else {
        d = opendir("/sys/bus/pci/devices");
        if (!d) {
            perror("opendir /sys/bus/pci/devices");
            return 1;
        }
        while ((de = readdir(d)) != NULL) {
            unsigned vid = 0, did = 0;

            if (de->d_name[0] == '.')
                continue;
            if (read_hex_file(de->d_name, "vendor", &vid) ||
                read_hex_file(de->d_name, "device", &did))
                continue;
            if (vid == EP_VENDOR_ID && did == EP_DEVICE_ID) {
                check_one(de->d_name);
                found = 1;
            }
        }
        closedir(d);
    }

    printf("\n== 驱动与设备节点 ==\n");
    printf("    /sys/module/anlogic_pci      : %s\n",
           access("/sys/module/anlogic_pci", F_OK) == 0 ? "已加载"
                                                        : "未加载");
    printf("    /dev/ANLOGIC-PCI0_control    : %s\n",
           access("/dev/ANLOGIC-PCI0_control", F_OK) == 0 ? "存在" : "不存在");
    printf("    /dev/ANLOGIC-PCI0_user       : %s\n",
           access("/dev/ANLOGIC-PCI0_user", F_OK) == 0 ? "存在" : "不存在");

    if (!found) {
        printf("\n!! 没找到 %04x:* 设备 —— EP 未枚举\n", EP_VENDOR_ID);
        printf("   排查：FPGA 是否已配置/固化？12V 供电？转接卡插紧？\n");
        printf("   重扫：echo 1 > /sys/bus/pci/rescan\n");
        printf("   日志：dmesg | grep -iE 'pcie.*(link|fail|error)'\n");
        return 1;
    }

    printf("\n链路与 BAR 若符合期望 = 硬件/建链正常；/dev 节点需要先 insmod anlogic_pci.ko。\n");
    return 0;
}
