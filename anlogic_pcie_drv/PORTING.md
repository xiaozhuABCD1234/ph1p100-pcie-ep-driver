# anlogic_pcie_drv 移植到 Linux 7.2（RK3588 / Orange Pi 5 Max）改动记录

上游来源：米联客 MLK-F9P-CA05 资料包 `11_例程/11_3_高速接口与PCIe/12_pcie_gen2x2/driver/ph1p_pcie_driver/`
（安路官方驱动，代码注释年份 2020/2023，本质是 Xilinx XDMA 的 libxdma 移植版）。

## 编译方式（在开发机 r9000p 上交叉编译）

```sh
make -C ~/Projects/orangepi-5-max-bsp/third_party/linux \
     M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
     KCFLAGS="-Wno-error" modules
```

`KCFLAGS="-Wno-error"`：7.x 内核把若干 warning 当错误（老代码里有未初始化变量、未使用函数等），
这是厂商驱动的常规处理；真正的 API 错误已逐条修掉，见下。

## 必须改的 API（否则编译不过）

| 位置 | 老写法 | 新写法 | 原因 |
|---|---|---|---|
| anlogic_pci_lib.c `XPCIe_exit` | `pci_free_consistent(pdev, ...)` | `dma_free_coherent(&pdev->dev, ...)` | 老 pci DMA API 已删除；同时去掉同函数里先 `kfree` 再 free 的双重释放（这两个 buffer 全文件从未分配，属遗留 XBMA 死代码） |
| anlogic_pci_lib.c `set_dma_mask` | `pci_set_dma_mask()` / `pci_set_consistent_dma_mask()` | `dma_set_mask(&pdev->dev, ...)` / `dma_set_coherent_mask(&pdev->dev, ...)` | 同上 |
| anlogic_pci_lib.c `engine_init` | `u32 val;` 然后 `if (val & 0x8000U)`（读 identifier 那行被注释掉） | `u32 val = 0;` | 未初始化变量，7.x 下是编译错误（-Werror=uninitialized）；置 0 即 MM 模式 |
| anlogic_pci_cdev.c `anlogic_cdev_init` | `class_create(THIS_MODULE, NAME)` | `class_create(NAME)` | 6.4 起 `class_create()` 去掉 owner 参数 |
| cdev_ctrl.c mmap 回调 | `vma->vm_flags |= VMEM_FLAGS;` | `vm_flags_set(vma, VMEM_FLAGS);` | 6.3 起 vm_flags 只读 |

## 本移植新增的行为

1. **`interrupt_mode` module 参数恢复 + 默认改为 1（MSI）**：原代码把 `module_param()` 注释掉了，
   写死 `interrupt_mode = 2`（Legacy），而本 EP 的 INTx 是断的（`Interrupt: pin A routed to IRQ 0`），
   导致中断完全不可用。现在可 `insmod anlogic_pci.ko interrupt_mode={0|1|2}`。
2. **用户 BAR 自动识别**：原 `map_bars()` 里识别 user BAR 的那段被注释掉，`user_bar_idx` 永远是 -1，
   `/dev/ANLOGIC-PCI0_user` 不会创建。补回：挑第一个「非 config BAR、非 IO、已映射」的 mem BAR。
   本板实测选中 **BAR2**（1MB）。

## 板侧落位与加载

```sh
# 开发机
sudo install -m 0644 anlogic_pci.ko ~/Projects/orangepi-5-max-bsp/rootfs/nfs/lib/modules/7.2.0-dirty/updates/
sudo depmod -b ~/Projects/orangepi-5-max-bsp/rootfs/nfs 7.2.0-dirty
# 板上
insmod /lib/modules/7.2.0-dirty/updates/anlogic_pci.ko
```

## 实测结果（2026-10-15）

- 加载后创建 `/dev/ANLOGIC-PCI0_control`（507:1）与 `/dev/ANLOGIC-PCI0_user`（507:0，user bar=2）。
- 8 个引擎枚举：`0-H2C0..3-MM` / `0-C2H0..3-MM`（offset 0x0/0x100/0x200/0x300 与 0x1000/0x1100/0x1200/0x1300）。
- **PIO 验证通过**：`./pci_bar 0 0x0 0x12345678` → 读回 `0x12345678`。
- ⚠️ 仍是 legacy 中断模式：EP 没上报 MSI/MSI-X 能力，中断这条线要么 FPGA 侧补，要么走轮询。
- ⚠️ 待查：BAR2 与 BAR0 在同一 offset 读到同一个值，需对照 FPGA 侧寄存器表确认映射是否为镜像。
