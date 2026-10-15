# PH1P100 PCIe 端点（EP）Linux 驱动

安路 PH1P100 FPGA 作 PCIe EP、RK3588 作 RC。本仓库是 RC 侧（主机侧）的驱动与用户态程序。
驱动由厂商 `anlogic_pci` 移植而来，适配内核 7.x，API 改动记录在 `anlogic_pcie_drv/PORTING.md`。

## 目录

- `anlogic_pcie_drv/` —— 内核模块源码（字符设备 + 引擎管理 + 控制/用户 BAR 节点）
- `app/` —— 用户态程序（`cam_grab.c` 抓帧存 PGM；`pci_bar.c` 直接 mmap 寄存器窗口，`cdev_ctrl.h` 控制节点协议）
- 厂商原版驱动（`anlogic_pci`）不在本仓库内：源码在原厂 SDK 与 `~/Projects/.archive-ph1p-*/vendor_driver_original/`，
  只在需要对照移植改动（见 `PORTING.md`）时取用，避免两份代码并存互相分叉
- `tools/opi_pcie_check.sh` —— 板上 PCIe 链路与 EP 自检脚本

## 交叉编译

```
make -C <内核树> M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- KCFLAGS="-Wno-error" modules
```

编译产物落在源码目录，已在 .gitignore 中排除。

## 加载与设备节点

`insmod anlogic_pci.ko` 后：
- `/dev/ANLOGIC-PCI0_control` —— 控制节点，8 个引擎、DMA/PIO 模式切换
- `/dev/ANLOGIC-PCI0_user` —— 用户 BAR 窗口（当前绑到 BAR2）

## 寄存器表（与 FPGA 工程对应）

无绝对路径的寄存器表以 FPGA 仓库 `fpga/rtl/ep_side/img/ep_bar_win.v` 为准，本仓库只列使用约定：

| 偏移 | 含义 |
| --- | --- |
| 0x40 | MAGIC，应为 0x50434946 |
| 0x44 | CTRL：bit0 enable、bit1 trigger（写 1 出单帧，下一拍自动清）、[3:2] pattern、bit4 cam_sel（1 = 相机写源） |
| 0x48 | STATUS |
| 0x4C | 帧计数 |
| 0x50/0x54 | 宽/高（640×480） |
| 0x58 | 有效数据长度（307200） |
| 0x5C/0x60 | 读指针 / 数据口 |
| 0x64/0x68 | 版本 / 时钟 MHz |
| 0x6C | SCRATCH（写读回环） |
| 0x80~0x98 | cam_stat[223:0]：A/B 状态字（0x80/0x84）、A/B 帧计数、A/B 像素组计数、I2C 忙/上电计数 |
| 0x9C | MIPI IDELAY tap 控制（默认 0x6e = manual_en + tap 46）；不是 cam_stat[255:224] |

访问约定（都是实测结论，不是猜测）：
- 每次读**不超过 4 个 dw**：FPGA 侧完成包长度未夹紧，超过会静默出错；
- 完成包数据在 `[255:224]` 起，主机小端读取需**逐 dw 字节翻转**（`bswap32`）；
- 寄存器读数在未加翻转时会读成逆序（MAGIC 会读成 0x46494350）。

## 抓帧工具 `app/cam_grab.c`

从 BAR0 寄存器表抓一帧 640x480 灰度图存 PGM，测试图源 / 相机源二选一；访问严格按
`doc/panic_risk_audit.md` 的规约（先读 0x40 MAGIC 自检、只碰登记过的 offset、单次单 dw、
任一读拿到 `0xffffffff` 立即熔断退出），发 MMIO 前先用 sysfs 的 `vendor:device` 认设备
（`1edb:abcd`，不是 PH1P100 EP 就拒绝）。

```sh
# 交叉编译（板子上直接跑，静态链接）
aarch64-linux-gnu-gcc -static -O2 -Wall -Wextra -o cam_grab app/cam_grab.c

# 板上：相机源（链路没锁会打印状态字逐位解码，退出码 3，不写图）
./cam_grab -s cam frame_cam.pgm
# 板上：测试图源（单帧触发 CTRL=[3:2]|bit1，抓完帧缓存静止，不会被边写边读撕裂）
./cam_grab -s pattern frame_pat.pgm
# 离线（不碰硬件）：状态字解码 / 自证主机侧像素映射与自检逻辑
./cam_grab --decode 0xcdee0020
./cam_grab --selftest
```

- 选项：`-s cam|pattern`（默认 cam）、`-d /sys/.../resource0`、`-t 超时ms`、`--help`。
  退出码：0 成功 / 1 用法 / 2 设备不可用 / 3 相机链路未就绪（不写图）/ 4 等不到新帧（不写图）/
  5 图已写出但校验不过 / 6 输出文件错误。
- 帧缓存布局是「字 0 = 帧号 + 76799 个像素字」：PGM 前 307196 个像素来自帧缓存，末尾 4 个
  像素（x=636..639, y=479）RTL 里就不存（`fpga/rtl/ep_side/img/cam_fb_writer.v:47-49`），工具填 0 并打印说明。
- 相机源的 `DATA_LEN` 是常量 307200（`cam_fb_writer.v:116`），判「有没有出帧」只能看 `FRAME_CNT`。

## 已知问题

- EP 没有 MSI/MSI-X 能力（capability 链里只有 Express/PM/Vendor/ARI/AER），帧到达只能靠轮询帧计数；
- 未走 DMA，PIO 实测 4 字节读 2.6 MB/s、8 字节读 5.2 MB/s，640×480 灰度帧约 17 fps 上限；
- FPGA 侧热重配置后主机 BAR 全部读回 0xffffffff，**必须重启 RK3588 重新枚举**才能恢复 Gen2 x2；
- BAR0/BAR2/BAR4 都是 1MB 且当前共享同一个窗口（4KB 别名），子字（<4 字节）写的语义未定义，请只用 32 位对齐访问；
- BAR2 与 BAR0 同偏移读回同值的原因尚未查清（现有测试都走 resource0 绕过）。

## 已验证

PCIe 图像通路（寄存器 + 640×480 整帧逐字比对 0/76800 字不符）在 RK3588 上实测通过。
