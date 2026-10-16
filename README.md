# PH1P100 PCIe 端点（RK3588 主机侧）驱动

安路 PH1P100 FPGA 作 PCIe EP、RK3588 作 RC 时，RC 侧的内核模块与用户态程序。
由厂商 `anlogic_pci` 驱动移植而来，原先只做 PIO 寄存器访问；现在另加了一层「收帧缓冲」
（dma_alloc_coherent 分配、物理地址/序号经 ioctl 给用户态、FPGA 侧拿它当 MWr 目标），
真正的收帧等 FPGA 接通后再验。
探测时驱动还会把「下 bit / 重启之后必须人工补」的三步做完（写 0x44 CTRL 使能位、
按需重触发一次传感器初始化、把收帧缓冲的 DMA 地址填进推帧目标与门铃寄存器），
失败只报错不上报 probe 失败，状态与手动重触发由三条链路 ioctl 暴露（见下面）。
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
| `src/anlogic_pci_link.c` | 相机链路 / 推帧：探测时初始化（使能位、推帧与门铃地址、按需重触发）、状态查询、手动重触发、地址复核/重设 |
| `src/anlogic_pci.h` | 驱动内部头（三层共用） |
| `include/anlogic_pci_ioctl.h` | ioctl 协议，驱动与用户态共用的唯一一份 |
| `app/rs/` | 用户态（Rust，cargo 工程 `anlogic_pci_tools`，五个 bin）：`cam_grab` 抓帧（PIO）、`frame_grab` 从收帧缓冲取帧、`link_ctl` 查/改链路与推帧状态、`pci_bar` 单 dw 读写、`pcie_check` 链路自检 |

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
`/dev/ANLOGIC-PCI0_frame`（**A 路**收帧缓冲，只读 mmap + 帧就绪 ioctl）、
`/dev/ANLOGIC-PCI0_frame_b`（**B 路**收帧缓冲，命令字与 A 路完全相同）。

收帧缓冲是**两路**的（`frame_streams` 默认 2；置 1 就退回单路老布局，那时只有 `_frame`）：
缓冲 = 两套完全相同的「槽区 + mailbox 页」平铺，A 路在前、B 路在后。
`ANLOGIC_IOCFRAMEINFO` 报的 `phys_addr` 是该路的起点、`mb_off` 是该路 mailbox 在整块缓冲
里的偏移、`size` 是整块缓冲（mmap 映射的也是整块，一次映射两路都能看）。
**用户态定位本路槽的口径**：路首偏移 = `mb_off - stride*slots`（A 路 = 0），
槽地址 = 映射基址 + 路首偏移 + 槽号×stride。

### 装到板子上（要覆盖掉厂商版，不能只 insmod）

这块板子的 rootfs 里**已经预装了一个厂商版 `anlogic_pci.ko`**，位置是
`/usr/lib/modules/$(uname -r)/updates/`。它带同样的 PCI ID 别名，所以开机时会被
自动加载，把手动 `insmod` 的移植版静默顶掉 —— 表现是**相机通路整个不可用**，
但不会报任何错。

症状：三个节点里只出现 `_control` 和 `_user`，**没有 `/dev/ANLOGIC-PCI0_frame`**
（收帧缓冲不分配、推帧也不初始化）；`lsmod` 里 size 显示 36864。

判别手上这个 `.ko` 是哪个版本：

```sh
lsmod | grep anlogic_pci                       # 36864 = 厂商版，45056 = 移植版
strings anlogic_pci.ko | grep -c create_xcdev  # 厂商版 4，移植版 0
```

所以装到板子上要覆盖那个路径并 `depmod`，而不是只把文件拷到某个目录再 `insmod`：

```sh
cp anlogic_pci.ko /usr/lib/modules/$(uname -r)/updates/
depmod -a
rmmod anlogic_pci && modprobe anlogic_pci      # 之后开机就是这个版本了
ls /dev/ANLOGIC-PCI0_*                         # 必须出现 _frame，否则还是厂商版
```

## 收帧缓冲（DMA）

驱动用 `dma_alloc_coherent` 分配一块物理连续、页对齐的缓冲给 FPGA 当 MWr 目标，内存布局、
mailbox 字段与握手约定写在 `include/anlogic_pci_ioctl.h`（FPGA 侧必须按那份布局写）：
每写完一帧先填槽头、最后更新 `mailbox.seq`（序号那笔 MWr 不能带 Relaxed Ordering）。
主机侧只轮询序号判就绪（本 EP 只有 Legacy 中断，暂不做 MSI 唤醒）。

缓冲区里槽区在前（槽 n 的 MWr 目标 = `phys_addr + stride*n + hdr_bytes`，即槽头之后的像素区起点；
槽头由驱动写、不能被帧数据盖掉），
mailbox 描述符在槽区之后的一整页里，位置用 ioctl 报出来的 `mb_off`，不要按常量猜。

- 规模由 module param 定：`frame_width`（3840）/`frame_height`（2160）/`frame_slot_bytes`（0 = 宽×高 = 8294400）/`frame_slots`（默认 1）；
  默认单槽、**每路一个固定目标地址**：推帧引擎只往主机给的那一个地址写（驱动探测时按路
  写好 0xB0/0xB4 与 0x100/0x104，不逐帧轮换），多槽下用户态按序号取槽会取到没数据的槽；
- 两路同时在推：CTRL 的 bit6 = A 路使能、bit8 = B 路使能，探测时两路都置上（本板实测 `0x44 = 0x161`）。
  老映像没有 0x100~0x10C 那组寄存器时，驱动打印一条明确警告并**退化成只跑 A 路**，不报错；
- ioctl：`ANLOGIC_IOCFRAMEINFO` 报物理地址、大小、几何、`mb_off` 与序号/计数；`ANLOGIC_IOCFRAMEWAIT` 等帧（`timeout_ms=0` 只查不等）；
- 推帧引擎只往门铃写「已推帧数」（不写槽头、不碰 mailbox.seq/count，见 `include/anlogic_pci_ioctl.h`），
  这三样由驱动在轮询时补齐，所以用户态照旧只读 `mailbox.seq`；
- 用户态：`./frame_grab --info` 只看缓冲参数，`./frame_grab -o f.pgm` 取最新一帧；
- FPGA 还没接上时 `mailbox.seq` 恒 0，`frame_grab` 打印「无新帧」并以退出码 4 返回（不写文件）。

## 收 4K（相机整帧直通 DDR3）

FPGA 侧新增了「相机 4K 整帧直通 DDR3」这条路（见 `fpga/README.md`），主机侧只需把收帧
缓冲按 4K 分配，其余代码不动 —— 几何、物理地址、门铃都由探测时那套 ioctl 报出来：

```sh
insmod anlogic_pci.ko frame_width=3840 frame_height=2160 frame_slots=1
# frame_slot_bytes 留 0 ⇒ 用 width*height = 8294400 字节/槽；frame_streams 默认 2
ls /dev/ANLOGIC-PCI0_frame /dev/ANLOGIC-PCI0_frame_b   # 两个都要出现
./frame_grab --info                   # A 路：应为 3840x2160、slot_bytes=8294400
./frame_grab -o a.pgm                 # A 路 8.29MB PGM
./frame_grab -d /dev/ANLOGIC-PCI0_frame_b -o b.pgm     # B 路
# 两路同时看（各起一个实例，端口分开）：
#   ./display-stream --source device --device /dev/ANLOGIC-PCI0_frame   --bayer gbrg --auto --port 8098 &
#   ./display-stream --source device --device /dev/ANLOGIC-PCI0_frame_b --bayer gbrg --auto --port 8099 &
```

双路实测（Gen2 ×2）：两路各 ≈7.85fps（= 传感器真实帧率）、各 64800 笔 MWr/帧、
`0xC8`/`0x118` 错误码恒 0、`0xC0`/`0x110` 两个已推帧数同时在涨；两个 display-stream
实例同跑各 ≈3fps（此时瓶颈在 RK3588 侧 MJPEG 编码，不在链路）。

要点与限制：

- **一帧 8294400 字节来自一次 `dma_alloc_coherent`**：8MB 级的一致性分配在 RK3588 上
  要求 CMA 够大（`dmesg | grep -i cma`）。分配失败会在 probe 里打印
  `dma_alloc_coherent(...) 失败`，dmesg 里一眼能看到。
- **`frame_slots` 必须留 1**：FPGA 的推帧引擎只往主机给的那一个目标地址写
  （探测时填的槽 0 像素区起点），多槽只有槽 0 会收到帧。
- 几何不用主机操心：`mailbox.width/height/slot_bytes` 与 `ANLOGIC_IOCFRAMEINFO`
  都是按 module param 算出来的，默认就是相机整帧 3840×2160（8294400 字节/槽），
  用户态照旧只轮询 `mailbox.seq`。`ANLOGIC_FRAME_MAX_SLOT_BYTES` 是 64MiB，
  4K 远在上限之内。
- **推帧引擎的每帧字节数由 FPGA 侧现算**，所以「驱动报的几何」和「FPGA 里那一帧的
  字节数」必须一致（默认两边都是 8294400）。这个数必须是 4KB 的整数倍
  （8294400 = 2025×4096）—— 换别的几何时这条要自己保证。
- **取帧时的「一致性」告警不是故障**：每路只有**一个**槽、推帧目标地址固定不轮换，
  而 FPGA 按 ≈7.85fps 一直往这个槽里写 —— 用户态正在读第 N 帧时，第 N+1 帧可能已经
  开始覆盖它。`frame_grab` 会核对槽头序号/flags 与「读的期间 mailbox.seq 有没有变」，
  撞上就打印 `一致性 **不通过**` 并提醒别当可信帧用（图照写，供排查用）。
  实测连续取 4 次两路都通过；要 100% 稳，取帧前把推帧使能关一下（`0x44` 清 bit6/bit8）
  再取，或者接受"偶发重取一次"。
- 4K 帧率目前仍受传感器限制（约 7.85fps，见 `fpga/README.md`），
  所以 PCIe/DDR 都不是瓶颈；取一帧 PGM 8.29MB，`frame_grab` 写文件本身也要点时间。

## 最小用法

```sh
# 先 cd 到构建产物目录：板子用 app/rs/target/aarch64-unknown-linux-gnu/release，本机用 app/rs/target/release
./pcie_check                     # 链路/枚举/BAR/节点自检，只读 sysfs，不需要 root
./pci_bar 0 0x40                 # 读 BAR0 + 0x40：MAGIC 应为 0x50434946（小端读回需 bswap）
./pci_bar 0 0x6C 0x12345678      # 往 BAR0 + 0x6C（SCRATCH）写
./cam_grab -s cam frame.pgm      # 抓一帧测试图灰度存 PGM（走 BAR0 的 PIO，慢；几何看 0x50/0x54）
./frame_grab --info              # 看收帧缓冲的物理地址/大小/几何/序号，不取帧
./frame_grab -o dma_frame.pgm    # 从收帧缓冲取最新一帧存 PGM（FPGA 未接通时退出码 4 = 无新帧）
./link_ctl                       # 查链路与推帧状态（是否锁/是否初始化/已推帧数/错误码/地址）
./link_ctl retrig -t 2000        # 手动重触发一次相机初始化并等锁定
./link_ctl push                  # 复核推帧目标与门铃地址（带 --dst-lo/--db-lo 等参数就是重设）
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
- 推帧每路只往一个固定主机地址写：探测时按路写好 A 的 0xB0/0xB4、B 的 0x100/0x104
  （= 各自那一路缓冲的槽 0 像素起点）与门铃字（A 0xB8/0xBC、B 0x108/0x10C），之后不逐帧
  轮换，所以 `frame_slots` 默认 1。要换地址用 `link_ctl push --dst-lo/--dst-hi/--db-lo/--db-hi`
  （A 路）。
- 链路/推帧的 ioctl 都在控制节点上：`ANLOGIC_IOCLINKSTATUS`（查）、
  `ANLOGIC_IOCLINKRETRIG`（手动重触发传感器初始化）、`ANLOGIC_IOCPUSHCFG`（复核/重设 A 路
  推帧目标与门铃地址）、`ANLOGIC_IOCPUSHCFGB`（同样的事，B 路，打到 0x100~0x10C）。
  加 B 路没有动老结构体：给老结构体加字段会改变 `sizeof`，而 `_IOWR` 的编号里含 `sizeof`，
  那等于改了老命令字的 ABI。
- FPGA 热重配置后主机 BAR 全部读回 `0xffffffff`，必须重启 RK3588 重新枚举才能恢复 Gen2 x2。
