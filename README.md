# PH1P100 PCIe 端点（RK3588 主机侧）驱动

安路 PH1P100 FPGA 作 PCIe EP、RK3588 作 RC 时，RC 侧的内核模块与用户态程序。
由厂商 `anlogic_pci` 驱动移植而来，原先只做 PIO 寄存器访问；现在另加了一层「收帧缓冲」
（dma_alloc_coherent 分配、物理地址/序号经 ioctl 给用户态、FPGA 侧拿它当 MWr 目标），
真正的收帧等 FPGA 接通后再验。
MMIO 访问规约（先读 BAR0+0x40 验 MAGIC、单笔不超过 4 个双字、读到 0xffffffff 立即
熔断）在两条路径上各自落实：控制节点 `/dev/ANLOGIC-PCI0_control` 的 ioctl 由内核
`src/anlogic_pci_ctrl.c` 逐笔校验；`/dev/ANLOGIC-PCI0_user` 的 mmap 只放 offset 0 的
一页裸窗口，逐 offset 白名单与门禁由用户态自持。`cam_grab` 为性能刻意绕过驱动、直接
mmap sysfs 的 `resource0`（pci-sysfs 的 mmap 不查资源是否已被驱动 claim），因此它自带
同一套校验，不经内核校验。

## 目录

| 路径 | 内容 |
| --- | --- |
| `src/anlogic_pci_main.c` | 模块与 PCI 驱动骨架（probe / remove / AER 错误处理） |
| `src/anlogic_pci_hw.c` | 资源与寄存器访问层（BAR 映射、区间、DMA 掩码、中断、通道） |
| `src/anlogic_pci_frame.c` | 收帧缓冲层（dma_alloc_coherent 分配/释放、帧就绪轮询、取帧节点的 ioctl 与只读 mmap） |
| `src/anlogic_pci_chrdev.c` | 字符设备注册层（class / char 区域、节点创建与销毁） |
| `src/anlogic_pci_ctrl.c` | 控制与用户节点的 file_operations（ioctl、mmap） |
| `src/anlogic_pci.h` | 驱动内部头（三层共用） |
| `include/anlogic_pci_ioctl.h` | ioctl 协议，驱动与用户态共用的唯一一份 |
| `app/rs/` | 用户态（Rust，cargo 工程 `anlogic_pci_tools`，四个 bin）：`cam_grab` 抓帧（PIO）、`frame_grab` 从收帧缓冲取帧、`pci_bar` 单 dw 读写、`pcie_check` 链路自检 |

厂商原始驱动不在本仓库（在 `~/Projects/.archive-ph1p-*/vendor_driver_original/`），
只在需要对照移植改动时取用，避免两份代码并存分叉。

## 交叉编译

内核模块（内核树外构建，产物 `anlogic_pci.ko` 落在仓库根）：

```sh
make -C ~/Projects/orangepi-5-max-bsp/third_party/linux \
     M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
```

用户态（Rust，四个 bin 目标；`.cargo/config.toml` 里指定 `linker = "aarch64-linux-gnu-gcc"`，
静态链接，产物在 `app/rs/target/<target>/release/`）：

```sh
cd app/rs
cargo build --release --target aarch64-unknown-linux-gnu   # 交叉编译到板子（aarch64）
cargo build --release                                      # 本机原生（冒烟测试用）
```

## 加载

```sh
insmod anlogic_pci.ko        # 中断方式：interrupt_mode={0 MSI-X | 1 MSI(默认) | 2 Legacy}
rmmod anlogic_pci
```

节点：`/dev/ANLOGIC-PCI0_control`（控制/寄存器）、`/dev/ANLOGIC-PCI0_user`（用户 BAR 窗口，本板 = BAR2）、
`/dev/ANLOGIC-PCI0_frame`（收帧缓冲，只读 mmap + 帧就绪 ioctl）。

## 收帧缓冲（DMA）

驱动用 `dma_alloc_coherent` 分配一块物理连续、页对齐的缓冲给 FPGA 当 MWr 目标，内存布局、
mailbox 字段与握手约定写在 `include/anlogic_pci_ioctl.h`（FPGA 侧必须按那份布局写）：
每写完一帧先填槽头、最后更新 `mailbox.seq`（序号那笔 MWr 不能带 Relaxed Ordering）。
主机侧只轮询序号判就绪（本 EP 只有 Legacy 中断，暂不做 MSI 唤醒）。

缓冲区里槽区在前（槽 n 的 MWr 目标 = `phys_addr + stride*n`，槽头之后才是像素），
mailbox 描述符在槽区之后的一整页里，位置用 ioctl 报出来的 `mb_off`，不要按常量猜。

- 规模由 module param 定：`frame_width`（640）/`frame_height`（480）/`frame_slot_bytes`（0 = 宽×高 = 307200）/`frame_slots`（默认 4，多帧余量）；
- ioctl：`ANLOGIC_IOCFRAMEINFO` 报物理地址、大小、几何、`mb_off` 与序号/计数；`ANLOGIC_IOCFRAMEWAIT` 等帧（`timeout_ms=0` 只查不等）；
- 用户态：`./frame_grab --info` 只看缓冲参数，`./frame_grab -o f.pgm` 取最新一帧；
- FPGA 还没接上时 `mailbox.seq` 恒 0，`frame_grab` 打印「无新帧」并以退出码 4 返回（不写文件）。

## 最小用法

```sh
# 先 cd 到构建产物目录：板子用 app/rs/target/aarch64-unknown-linux-gnu/release，本机用 app/rs/target/release
./pcie_check                     # 链路/枚举/BAR/节点自检，只读 sysfs，不需要 root
./pci_bar 0 0x40                 # 读 BAR0 + 0x40：MAGIC 应为 0x50434946（小端读回需 bswap）
./pci_bar 0 0x6C 0x12345678      # 往 BAR0 + 0x6C（SCRATCH）写
./cam_grab -s cam frame.pgm      # 抓一帧 640x480 灰度存 PGM（走 BAR0 的 PIO，慢）
./frame_grab --info              # 看收帧缓冲的物理地址/大小/几何/序号，不取帧
./frame_grab -o dma_frame.pgm    # 从收帧缓冲取最新一帧存 PGM（FPGA 未接通时退出码 4 = 无新帧）
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

- MMIO 门禁：控制节点的 ioctl 由驱动逐笔校验（BAR0 先读 0x40 验 MAGIC、单笔不超过
  4 个双字、读到 `0xffffffff` 立即 `-EIO` 熔断）；`_user` 节点的 mmap 与 `cam_grab` 直连
  sysfs `resource0` 这两条路径由用户态自持同样的校验。
- 几个用户态工具的 flock 只锁各自打开的文件（`cam_grab` 锁 sysfs `resource0`、
  `frame_grab` 锁 `_frame`、`pci_bar` 不锁），不是跨工具的一次一个进程；`cam_grab` 与
  `pci_bar` 可以同时跑。驱动侧只用 `hw->lock` 串行化同一实例上的 ioctl/mmap。
- 单次 MMIO 读不超过 4 dw，只用 32 位对齐访问：FPGA 侧完成包长度未夹紧，长读会静默错数据。
- FPGA 热重配置后主机 BAR 全部读回 `0xffffffff`，必须重启 RK3588 重新枚举才能恢复 Gen2 x2。
