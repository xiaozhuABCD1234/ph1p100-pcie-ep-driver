# PH1P100 PCIe 端点（EP）Linux 驱动

安路 PH1P100 FPGA 作 PCIe EP、RK3588 作 RC。本仓库是 RC 侧（主机侧）的驱动与用户态程序。
驱动由厂商 `anlogic_pci` 移植而来，适配内核 7.x，API 改动记录在 `anlogic_pcie_drv/PORTING.md`。

## 目录

- `anlogic_pcie_drv/` —— 内核模块源码（字符设备 + 引擎管理 + 控制/用户 BAR 节点）
- `app/` —— 用户态示例（`pci_bar.c` 直接 mmap 寄存器窗口，`cdev_ctrl.h` 控制节点协议）
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
| 0x44 | CTRL |
| 0x48 | STATUS |
| 0x4C | 帧计数 |
| 0x50/0x54 | 宽/高（640×480） |
| 0x58 | 有效数据长度（307200） |
| 0x5C/0x60 | 读指针 / 数据口 |
| 0x64/0x68 | 版本 / 时钟 MHz |
| 0x6C | SCRATCH（写读回环） |
| 0x80~0x94 | 摄像头 A/B 状态、帧数、像素脉冲 |
| 0x9C | MIPI IDELAY tap 控制（默认 0x6e = manual_en + tap 46） |

访问约定（都是实测结论，不是猜测）：
- 每次读**不超过 4 个 dw**：FPGA 侧完成包长度未夹紧，超过会静默出错；
- 完成包数据在 `[255:224]` 起，主机小端读取需**逐 dw 字节翻转**（`bswap32`）；
- 寄存器读数在未加翻转时会读成逆序（MAGIC 会读成 0x46494350）。

## 已知问题

- EP 没有 MSI/MSI-X 能力（capability 链里只有 Express/PM/Vendor/ARI/AER），帧到达只能靠轮询帧计数；
- 未走 DMA，PIO 实测 4 字节读 2.6 MB/s、8 字节读 5.2 MB/s，640×480 灰度帧约 17 fps 上限；
- FPGA 侧热重配置后主机 BAR 全部读回 0xffffffff，**必须重启 RK3588 重新枚举**才能恢复 Gen2 x2；
- BAR0/BAR2/BAR4 都是 1MB 且当前共享同一个窗口（4KB 别名），子字（<4 字节）写的语义未定义，请只用 32 位对齐访问；
- BAR2 与 BAR0 同偏移读回同值的原因尚未查清（现有测试都走 resource0 绕过）。

## 已验证

PCIe 图像通路（寄存器 + 640×480 整帧逐字比对 0/76800 字不符）在 RK3588 上实测通过。
