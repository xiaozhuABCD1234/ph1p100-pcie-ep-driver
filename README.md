# PH1P100 PCIe 端点（RK3588 主机侧）驱动

当前入口（2026-10-27）：队友先看 [A1U1 真样例交接包](handoff/rk-a1u1-b18-20261026/README.md)，保留已经成功的 RAW/R10F 收图和解码软件，再增加文件输入。原仓 `main` 的本次同步范围、配对协议及已知接收循环/CANCEL 局限见 [同步记录](docs/PROJECT_SYNC_20261027.md)；逐文件来源与 SHA 见 [复制清单](docs/sync-20261027-files.json)。下文保留历史构建与诊断说明，不能视为本次 RK 板验通过。


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
| `src/anlogic_pci_rka1.c` | RKA1 安全扩展（0x100..0x11C）：EXT_MAGIC/VERSION 门禁、one-shot ARM/ACK/CANCEL 与状态机自愈 |
| `src/anlogic_pci.h` | 驱动内部头（各层共用） |
| `include/anlogic_pci_ioctl.h` | ioctl 协议，驱动与用户态共用的唯一一份 |
| `app/rs/` | 用户态（Rust，cargo 工程 `anlogic_pci_tools`，六个 bin）：`cam_grab` legacy PIO 抓帧（当前位流默认拒绝，需 `--legacy`）、`frame_grab` 从收帧缓冲取帧、`link_ctl` 查/改链路与推帧状态、`pci_bar` 单 dw 读写、`pcie_check` 链路自检、`r10f_check` R10F 帧校验/抓帧 |
| `tools/` | 离线工具：`r10f_view.py`（dump/实时抓帧转 PNG）、`pcie_integration/r10f_strict.py`（R10F 严格校验）、`pcie_integration/r10f_tools_consistency.py`（Python/Rust 字段交叉检查） |
| `docs/`、`handoff/` | 协议合同与验收向量、队友交接包（入口见 `handoff/rk-a1u1-b18-20261026/README.md`） |

厂商原始驱动不在本仓库（在 `~/Projects/.archive-ph1p-*/vendor_driver_original/`），
只在需要对照移植改动时取用，避免两份代码并存分叉。

## 交叉编译

内核模块（内核树外构建，产物 `anlogic_pci.ko` 落在仓库根）：

```sh
make -C ~/Projects/orangepi-5-max-bsp/third_party/linux \
     M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
```

用户态（Rust，六个 bin 目标；`.cargo/config.toml` 里指定 `linker = "aarch64-linux-gnu-gcc"`，
静态链接，产物在 `app/rs/target/<target>/release/`）：

```sh
cd app/rs
cargo build --release --target aarch64-unknown-linux-gnu   # 交叉编译到板子（aarch64）
cargo build --release                                      # 本机原生（冒烟测试用）
```

## 加载

```sh
insmod anlogic_pci.ko        # 默认 use_irq=0：不申请中断，收帧轮询 mailbox
insmod anlogic_pci.ko use_irq=1 interrupt_mode=1   # 只有 FPGA 真会发 MSI 时才需要
rmmod anlogic_pci
```

节点：`/dev/ANLOGIC-PCI0_control`（控制/寄存器）、`/dev/ANLOGIC-PCI0_user`（用户 BAR 窗口，本板 = BAR2）、
`/dev/ANLOGIC-PCI0_frame`（收帧缓冲，只读 mmap + 帧就绪 ioctl）。

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

## RKA1 one-shot 与重新授权

- 探测时 `link_arm` 会做一次 ARM，所以上电后第一帧推完就会停在 HELD（这是有意的
  “第一帧额度”，不是漏 ACK）；`frame_grab` 能直接读到这一帧。
- 用户态要换新 session 重新授权时，`OP_CONFIGURE` 若看到 ARMED/BUSY/HELD/DRAINING，
  会先用自己的 session ACK 收回 IDLE（ACK 不匹配则退回 CANCEL），再写新的
  CAP_BYTES/SESSION_ID；因此探测留下的 HELD 不会把后续 `rka1_smoke --arm` 卡死。
- ARM 前仍会复核 `link_armed`、槽 0/门铃地址与 FPGA 寄存器回读；任何一项不符直接拒绝，
  不会把帧打到 0 或旧地址。

## 收帧缓冲（DMA）

驱动用 `dma_alloc_coherent` 分配一块物理连续、页对齐的缓冲给 FPGA 当 MWr 目标，内存布局、
mailbox 字段与握手约定写在 `include/anlogic_pci_ioctl.h`（FPGA 侧必须按那份布局写）：
每写完一帧先填槽头、最后更新 `mailbox.seq`（序号那笔 MWr 不能带 Relaxed Ordering）。
主机侧只轮询序号判就绪。当前 RTL 的 `tl_int_msireq` 恒 0，驱动默认
`use_irq=0`，既不申请 MSI/MSI-X 也不登记 legacy INTx；legacy 共享线没有可读的
状态/ACK 寄存器，无法判断中断归属，登记了反而会误吞同线中断。

缓冲区里槽区在前（槽 n 的 MWr 目标 = `phys_addr + stride*n + hdr_bytes`，即槽头 + 112B pad 之后的
R10F 数据起点 128；槽头由驱动写、不能被帧数据盖掉），
mailbox 描述符在槽区之后的一整页里，位置用 ioctl 报出来的 `mb_off`，不要按常量猜。

- 规模由 module param 定：`frame_width`（默认 1920）/`frame_height`（默认 1080）/`frame_slot_bytes`（默认 0 = 按几何推导 R10F 传输量，1920×1080 时 2592768；显式给出必须与推导值完全一致）/`frame_slots`（默认 1，且只接受 1）；
  默认单槽：推帧引擎只往主机给的那一个目标地址写（驱动探测时写好、不逐帧轮换），多槽下用户态按序号取槽会取到没数据的槽；
- ioctl：`ANLOGIC_IOCFRAMEINFO` 报物理地址、大小、几何、`mb_off` 与序号/计数；`ANLOGIC_IOCFRAMEWAIT` 等帧（`timeout_ms=0` 只查不等）。
  等帧最多 600 s，但驱动在两次轮询之间会放开 `hw->lock`，不会把同实例上的其它 ioctl/open/mmap 堵住；
- 推帧引擎只往门铃写「已推帧数」（不写槽头、不碰 mailbox.seq/count，见 `include/anlogic_pci_ioctl.h`），
  这三样由驱动在轮询时补齐，所以用户态照旧只读 `mailbox.seq`；
- 用户态：`./frame_grab --info` 只看缓冲参数，`./frame_grab -o f.pgm` 取最新一帧；
- FPGA 还没接上时 `mailbox.seq` 恒 0，`frame_grab` 打印「无新帧」并以退出码 4 返回（不写文件）。

## 收 4K（相机整帧直通 DDR3）

FPGA 侧新增了「相机 4K 整帧直通 DDR3」这条路（见 `fpga/README.md`），主机侧只需把收帧
缓冲按 4K 分配，其余代码不动 —— 几何、物理地址、门铃都由探测时那套 ioctl 报出来：

```sh
insmod anlogic_pci.ko frame_width=3840 frame_height=2160 frame_slots=1
# frame_slot_bytes 留 0 ⇒ 按 3840×2160 推导 R10F = 10371072 字节/槽
# （128B 头 + 10368000B packed RAW10 = 10368128，向上取整到 4KiB）
ls /dev/ANLOGIC-PCI0_frame            # 必须出现，否则还是厂商版驱动（见上）
./frame_grab --info                   # 应为 3840x2160、slot_bytes=10371072
./frame_grab -o f.pgm                 # 8.29MB PGM（8bit 灰度预览 3840×2160）
```

要点与限制：

- **一帧 10371072 字节来自一次 `dma_alloc_coherent`**：10MB 级的一致性分配在 RK3588 上
  要求 CMA 够大（`dmesg | grep -i cma`）。分配失败会在 probe 里打印
  `dma_alloc_coherent(...) 失败`，dmesg 里一眼能看到。
- **`frame_slots` 必须留 1**：FPGA 的推帧引擎只往主机给的那一个目标地址写
  （探测时填的槽 0 像素区起点），多槽只有槽 0 会收到帧。
- 驱动侧按几何推导槽长：`mailbox.width/height/slot_bytes` 与
  `ANLOGIC_IOCFRAMEINFO` 都来自 module param，用户态照旧只轮询
  `mailbox.seq`。`ANLOGIC_FRAME_MAX_SLOT_BYTES` 是 64MiB，4K 远在上限之内。
- **推帧引擎的每帧字节数必须与驱动推导值完全一致**：4K R10F 是 3840×2160 的
  10371072（含 128B 头和 4KiB 尾），1920×1080 是 2592768。旧 8bit 通路（640×480
  的 307200）不在当前默认几何内；如要用历史布局，请用历史位流/驱动，不要在当前
  R10F 位流上显式传 307200。
- 4K 帧率目前仍受传感器限制（约 14.5fps，见 `fpga/README.md`），
  所以 PCIe/DDR 都不是瓶颈；取一帧 PGM 预览 8.29MB，`frame_grab` 写文件本身也要点时间。

## 最小用法

```sh
# 先 cd 到构建产物目录：板子用 app/rs/target/aarch64-unknown-linux-gnu/release，本机用 app/rs/target/release
./pcie_check                     # 链路/枚举/BAR/节点自检，只读 sysfs，不需要 root
./pci_bar 0 0x40                 # 读 BAR0 + 0x40：MAGIC 应为 0x50434946（小端读回需 bswap）
./pci_bar 0 0x6C 0x12345678      # 往 BAR0 + 0x6C（SCRATCH）写
./frame_grab --info              # 看收帧缓冲的物理地址/大小/几何/序号，不取帧
./frame_grab -o dma_frame.pgm    # 从收帧缓冲取最新一帧存 PGM（FPGA 未接通时退出码 4 = 无新帧）
./link_ctl                       # 查链路与推帧状态（是否锁/是否初始化/已推帧数/错误码/地址）
./link_ctl retrig -t 2000        # 手动重触发一次相机初始化并等锁定
./link_ctl push                  # 复核推帧目标与门铃地址（--dst/--db 必须与驱动分配的槽 0/门铃完全一致，否则 -EINVAL）
```

## 移植要点（适配内核 7.x 的 API 改动）

- `pci_alloc_consistent()` / `pci_free_consistent()` → `dma_alloc_coherent()` / `dma_free_coherent()`；
  `pci_set_dma_mask()` / `pci_set_consistent_dma_mask()` → `dma_set_mask()` / `dma_set_coherent_mask()`；
- `class_create(THIS_MODULE, name)` → `class_create(name)`（6.4 起去掉 owner）；
- `vma->vm_flags |= VMEM_FLAGS` → `vm_flags_set(vma, VMEM_FLAGS)`（6.3 起 vm_flags 只读）；
- `access_ok()` 去掉第一个参数（5.0 起）；
- `struct pci_error_handlers` 用 `reset_prepare` / `reset_done`。

本板相关的行为改动：中断默认关闭（`use_irq=0`，收帧走 mailbox 轮询）；需要中断时
`use_irq=1` + `interrupt_mode={0 MSI-X | 1 MSI}`，legacy 仍不登记。用户 BAR 自动识别
（挑第一个非 config、非 IO、已映射的 mem BAR，本板选中 BAR2）。

## 日志约定

- 内核日志统一带层标签：`anlogic_pci: [main|hw|chrdev|ctrl|frame|link|rka1] ...`，
  由各文件的 `pr_fmt` 产生；看驱动日志用 `dmesg | grep anlogic_pci`。
- 级别：硬件/协议/寄存器回读失败、超时用 `pr_err`/`pr_warn`；状态变化（probe、链路
  初始化、卸载收尾）用 `pr_info`；每帧、每次轮询、以及用户传参错误用 `pr_debug`
  （后者不刷 dmesg，避免一个坏参数把 ring buffer 冲掉）。
- 时间一律打印真实毫秒（`ktime_ms_delta()`），不把轮询次数当毫秒；多行日志的续行
  与格式串对齐，且每条都以 `\n` 结尾。
- 用户态工具统一 `==` 标题、`!!` 错误、三空格缩进细节；错误同时给退出码，便于脚本判断。

## 注意

- MMIO 门禁：控制节点的 ioctl 由驱动逐笔校验（BAR0 先读 0x40 验 MAGIC、单笔不超过
  4 个双字、读到 `0xffffffff` 立即 `-EIO` 熔断）；`_user` 节点的 mmap 与 `cam_grab` 直连
  sysfs `resource0` 这两条路径由用户态自持同样的校验。
- 几个用户态工具的 flock 只锁各自打开的文件（`cam_grab` 锁 sysfs `resource0`、
  `frame_grab` 锁 `_frame`、`pci_bar` 不锁），不是跨工具的一次一个进程；`cam_grab` 与
  `pci_bar` 可以同时跑。驱动侧用 `hw->lock` 串行化同一实例上的 ioctl/mmap；唯一的例外是
  `ANLOGIC_IOCFRAMEWAIT`，它在每次轮询时短暂取锁、睡眠时放开。
- 单次 MMIO 读不超过 4 dw，只用 32 位对齐访问：FPGA 侧完成包长度未夹紧，长读会静默错数据。
- 推帧这一路只往一个固定主机地址写：探测时写好 0xB0/0xB4（= 收帧缓冲的 DMA 地址）
  与 0xB8/0xBC（= 缓冲内 mailbox 页的门铃字），之后不逐帧轮换，所以 `frame_slots` 默认 1。
  要换地址用 `link_ctl push --dst-lo/--dst-hi/--db-lo/--db-hi`（写了必回读复核）。
- 链路/推帧的三条 ioctl 都在控制节点上：`ANLOGIC_IOCLINKSTATUS`（查）、
  `ANLOGIC_IOCLINKRETRIG`（手动重触发传感器初始化）、`ANLOGIC_IOCPUSHCFG`（复核/重设
  推帧目标与门铃地址）；结构体与位定义都在 `include/anlogic_pci_ioctl.h`。
- FPGA 热重配置后主机 BAR 全部读回 `0xffffffff`，必须重启 RK3588 重新枚举才能恢复 Gen2 x2。
- `cam_grab` 是 T07 之前的 640×480 “测试图/相机 → fb[] → PIO” 工具；当前 RTL 的
  CTRL[4] 已保留（读回 0），硬件抓帧默认直接以 EX_DEV 拒绝、不碰 BAR；只有显式
  `--legacy` 才走旧路径（老位流调试用）。`--decode` / `--selftest` 仍可离线用。
  当前收帧请用 `frame_grab`（R10F）或 `tools/r10f_view.py`（PNG）。

## 代码风格

- C 源文件与头文件统一 **tab 缩进（8 列）**，格式化后 `checkpatch.pl --file` 为
  **0 error**；剩余 warning 主要是长行（中文日志/注释按字节计）和 ABI 头里的 typedef。
- 用户态 Rust 用 rustfmt 默认风格，Python 只用标准库；日志格式见上面的“日志约定”。
- 做完功能改动后至少跑：`make ... W=1 modules`、`cargo test --offline`、
  `r10f_strict.py selftest`、`r10f_tools_consistency.py`、`git diff --check`。
