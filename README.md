# PH1P100 PCIe 端点（RK3588 主机侧）驱动

安路 PH1P100 FPGA 作 PCIe EP、RK3588 作 RC 时，RC 侧的内核模块与用户态程序。
由厂商 `anlogic_pci` 驱动移植而来，只做 PIO 寄存器访问（未走 DMA）。
所有用户态可控的 MMIO 都在 ioctl/mmap 入口逐项校验，访问规约见
`doc/panic_risk_audit.md` —— 那些校验一条都不许放宽。

## 目录

| 路径 | 内容 |
| --- | --- |
| `src/anlogic_pci_main.c` | 模块与 PCI 驱动骨架（probe / remove / AER 错误处理） |
| `src/anlogic_pci_hw.c` | 资源与寄存器访问层（BAR 映射、区间、DMA 掩码、中断、通道） |
| `src/anlogic_pci_chrdev.c` | 字符设备注册层（class / char 区域、节点创建与销毁） |
| `src/anlogic_pci_ctrl.c` | 控制与用户节点的 file_operations（ioctl、mmap） |
| `src/anlogic_pci.h` | 驱动内部头（三层共用） |
| `include/anlogic_pci_ioctl.h` | ioctl 协议，驱动与用户态共用的唯一一份 |
| `app/` | 用户态：`cam_grab.c` 抓帧、`pci_bar.c` 单 dw 读写、`pcie_check.c` 链路自检 |
| `doc/panic_risk_audit.md` | 内核挂死风险审计与访问规约（只读参考） |

厂商原始驱动不在本仓库（在 `~/Projects/.archive-ph1p-*/vendor_driver_original/`），
只在需要对照移植改动时取用，避免两份代码并存分叉。

## 交叉编译

内核模块（内核树外构建，产物 `anlogic_pci.ko` 落在仓库根）：

```sh
make -C ~/Projects/orangepi-5-max-bsp/third_party/linux \
     M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
```

用户态（板子上直接跑，静态链接）：

```sh
aarch64-linux-gnu-gcc -static -O2 -Wall -Wextra -Iinclude -o cam_grab  app/cam_grab.c
aarch64-linux-gnu-gcc -static -O2 -Wall -Wextra -Iinclude -o pci_bar  app/pci_bar.c
aarch64-linux-gnu-gcc -static -O2 -Wall -Wextra -Iinclude -o pcie_check app/pcie_check.c
```

## 加载

```sh
insmod anlogic_pci.ko        # 中断方式：interrupt_mode={0 MSI-X | 1 MSI(默认) | 2 Legacy}
rmmod anlogic_pci
```

节点：`/dev/ANLOGIC-PCI0_control`（控制/寄存器）、`/dev/ANLOGIC-PCI0_user`（用户 BAR 窗口，本板 = BAR2）。

## 最小用法

```sh
./pcie_check                     # 链路/枚举/BAR/节点自检，只读 sysfs，不需要 root
./pci_bar 0 0x40                 # 读 BAR0 + 0x40：MAGIC 应为 0x50434946（小端读回需 bswap）
./pci_bar 0 0x6C 0x12345678      # 往 BAR0 + 0x6C（SCRATCH）写
./cam_grab -s cam frame.pgm      # 抓一帧 640x480 灰度存 PGM
```

## 移植要点（适配内核 7.x 的 API 改动）

- `pci_alloc_consistent()` / `pci_free_consistent()` → `dma_alloc_coherent()` / `dma_free_coherent()`；
  `pci_set_dma_mask()` / `pci_set_consistent_dma_mask()` → `dma_set_mask()` / `dma_set_coherent_mask()`；
- `class_create(THIS_MODULE, name)` → `class_create(name)`（6.4 起去掉 owner）；
- `vma->vm_flags |= VMEM_FLAGS` → `vm_flags_set(vma, VMEM_FLAGS)`（6.3 起 vm_flags 只读）；
- `access_ok()` 去掉第一个参数（5.0 起）；
- `struct pci_error_handlers` 用 `reset_prepare` / `reset_done`。

本板相关的行为改动：`interrupt_mode` 恢复为 module 参数且默认为 MSI；用户 BAR 自动识别
（挑第一个非 config、非 IO、已映射的 mem BAR，本板选中 BAR2）。

## 注意

- 单次 MMIO 读不超过 4 dw，只用 32 位对齐访问：FPGA 侧完成包长度未夹紧，长读会静默错数据。
- FPGA 热重配置后主机 BAR 全部读回 `0xffffffff`，必须重启 RK3588 重新枚举才能恢复 Gen2 x2。
