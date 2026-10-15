# PH1P100 PCIe EP（RK3588 RC 侧）「把内核搞挂」访问路径 · 只读审计报告

- 审计对象：`driver/anlogic_pcie_drv/`（全部 .c/.h + PORTING.md + Makefile）、`driver/app/`（pci_bar.c、cdev_ctrl.h）、
  RK3588 内核源码 `~/Projects/orangepi-5-max-bsp/third_party/linux/`（pcie-dw-rockchip.c、pcie/aer*.c、pci-driver.c、pci.c、
  pcie/err.c、pcie/portdrv.c、access.c、arch/arm64/mm/fault.c、arch/arm64/kernel/traps.c、DWC 控制器与 rk3588 设备树）、
  以及现有实测记录：`fpga/doc/code_review_pi_20261015.md`、`..._run2.md`、`driver/README.md`、`anlogic_pcie_drv/PORTING.md`。
- 取证方式：**只读**（read / grep / awk / wc / unzip -p 管道）。未修改任何代码或内核源码，未构建，未 insmod/rmmod，未 ssh 到板子，未下载/安装任何东西，未跑仿真。
- **本次审计执行的写入**：仅本文件 `driver/doc/panic_risk_audit.md`（及其父目录 `driver/doc/`，原先不存在）。
- **临时文件披露**：审计过程中为提取厂商文档 `fpga/doc/APUG081_...docx` 正文，曾把解包后的 XML 文本重定向到 `/tmp/apug081.txt`（约 57 KB），
  发现后已 `rm -f` 删除，该路径现不存在。除此之外没有其它文件被创建/修改/删除。
- 未运行项：**没有重跑任何板卡实验**，本报告所有“板卡行为”均来自仓库里已有的文字记录（README/PORTING/code_review）或代码/内核源码推导，
  逐条标注了「实测」/「代码证据」/「未确认」。板子的 panic dmesg 原始日志不在仓库内（已在整个工程与 `/tmp` 范围外搜索，未见），
  因此**无法把某次具体 panic 归因到下面某一条**，只能给出机制候选与排查方法（见 §2.4、§5）。

---

## 0 结论速览

1. **真正“打到 EP 不响应地址”的用户态/驱动路径，在现有代码里主要只有三条能实际发生**：
   ① `ANLOGIC_IOC*` 系列 ioctl（未经校验的 `bar_id` / `bar_offaddr` / `bar_len`）；
   ② `/dev/ANLOGIC-PCI0_user` 的 `mmap`（`psize` 整数下溢，可映射到 BAR 之外甚至物理内存）；
   ③ 绕过驱动的 `sysfs resourceN` mmap（历史测试用法，仓库里没有对应源码，README:56 只提了一句“现有测试都走 resource0 绕过”）。
2. **最容易把内核直接打死、而且用仓库自带示例就能触发的两条**都在 ioctl 路径：
   - `ANLOGIC_IOCBULKR/BULKW` 的循环索引越界 → **内核栈越界写/读**（`cdev_ctrl.c:257-258, 271-272`），bar_len 稍大即栈溢出（arm64 栈溢出 = `nmi_panic` = 必然 panic）；
   - `ANLOGIC_IOCCFGW/CFGR` **直接解引用用户指针**（`cdev_ctrl.c:282-288`）→ arm64（`CONFIG_ARM64_EPAN=y`）下必然
     `Unable to handle kernel access to user memory outside uaccess routines` oops。
3. **AER 本身不会 panic**：本内核 `drivers/pci/pcie/aer.c` 全文没有 `panic()/BUG()`；可纠正错误只打印+清状态，不可纠正走 `pcie_do_recovery()`。
   真正的 panic/卡死来自 AER 之外的三条路（§2.4）：PCIe 事务触发的 CPU 外部中止（同步 SEA→oops / 异步 SError→**panic**）、
   AER 恢复路径最长 60 s 的等待+总线/设备锁、以及上面 2 里的栈越界。
4. **现有驱动对 `bar_id` 完全没有边界检查**，而示例程序 `app/pci_bar.c:85` 的检查是 `bar_id > 6`（**off-by-one**，放过了 6），
   于是 `./pci_bar 6 0x0` 或 `./pci_bar 3 0x0`（不存在的 BAR）就能让内核在 `xdev->bar[]` 越界/NULL 指针上做 MMIO。
5. FPGA 侧加固的第一优先项不是“回固定值”，而是**保证 BAR 窗内每一个地址都必有完成包**；其次是完成包长度必须按实际服务长度夹紧（否则 >4 dw 读静默错数据），
   以及把 4DW（64 位 BAR）地址解码补上（否则 BAR2/BAR4 读回恒 `DEADBEEF`）。验证点与验证方法见 §4。

---

## 1 现有驱动/用户态里“会打到 EP 不响应地址”的访问路径（逐条 + 风险等级）

### 1.1 风险总表

| # | 位置（文件:行） | 访问形态 | 触发条件 | 后果（为什么） | 等级 |
|---|---|---|---|---|---|
| R1 | `anlogic_pcie_drv/cdev_ctrl.c:226,228`（IOCW）、`:239,241`（IOCR） | `xdev->bar[bar_id] + off` 后 `iowrite32/ioread32` | `bar_id` 未校验（`unsigned short`，0..65535）；`off` 未校验（32 位） | bar_id≥6 越界读结构体字段当指针（`anlogic_pci_lib.h:614` 只有 `bar[6]`）；bar 未映射时为 NULL；`off≥0x100000` 时越过 `pci_iomap` 长度（`anlogic_pci_lib.c:929`，按 BAR 长度 1 MB 映射）→ 内核态翻译/权限异常 → oops | **极高** |
| R2 | `cdev_ctrl.c:257-258`（BULKW）、`:271-272`（BULKR） | `for (i=bar_offaddr; i<bar_len; i++) iowrite32/`**`ioctl_bar_bulk.rdata[i]=ioread32(...)`** | 用户给出 `bar_len > MAX_DATA_SIZE(100)`（`cdev_ctrl.h:90,97-98`） | ① `rdata[i]/wdata[i]` 索引 i 从 bar_offaddr 起、上界 bar_len-1，数组只有 100 项 → **内核栈越界写（BULKR）/越界读（BULKW）**；② 循环次数 = bar_len-bar_offaddr，可到 ~4×10⁹ → 4G 次 MMIO 读/写、单次 ioctl 不返回；③ 地址只按 `i>>2` 前进 → 可覆盖 base 起 `bar_len/4` 字节（1 MB 窗扫描是 `bar_len=0x40000`，正好必然栈越界） | **极高** |
| R3 | `cdev_ctrl.c:313-346`（`bridge_mmap`） | `mmap(/dev/ANLOGIC-PCI0_user, ...)` | `vma->vm_pgoff` 给出的 `off` > BAR 长度 | `psize = end-start+1-off` 为无符号（resource_size_t）→ **下溢成极大值** → `vsize > psize` 检查（`:332`）失效 → `io_remap_pfn_range(phys>>PAGE_SHIFT)`（`:345`）把 `start+off` 起的任意物理页映射成 uncached 用户可读写；偏移正好落在 PCIe 14 MB 窗内但 EP 不认领时 → UR/超时；落在窗外的物理地址 → 用户态外部中止（SIGBUS）或直接读到别的物理内存 | **极高**（安全 + 崩溃） |
| R4 | `cdev_ctrl.c:282-288`（CFGW/CFGR） | `(*(cfgwr *)arg).reg/.value`、`*((uint32_t*)arg) = regx` | 任何一次该 ioctl 调用 | **用户指针在核心里被直接解引用**（没有 copy_from_user/copy_to_user）。arm64 `CONFIG_ARM64_EPAN=y`（`.config:564`）下 EL1 直接访用户地址 → `arch/arm64/mm/fault.c:659-666` → `die_kernel_fault("access to user memory outside uaccess routines")` → oops | **极高**（必现） |
| R5 | `cdev_ctrl.c:139-153`（`XPCIe_ReadCfgReg/WriteCfgReg`）+ `:281-289` | `pci_read/write_config_dword(pdev, byte, ...)`，`byte`/`reg` 用户可控 | 传任意 config offset（4 GB 范围） | 对下游 EP 的越界 config 读会生成 Type0/Type1 配置请求；EP 不应答 → 超时后硬件合成全 F（`drivers/pci/pci.c:1220-1226` 注释），一般能返回但是**白填 0xffffffff**，且 IE/AER 侧可能记 Unsupported Request；若 EP 侧 config 解码异常则可能长时间等待 | **高**（功能/错误注入面，非必然崩） |
| R6 | `app/pci_bar.c:85` + 驱动无校验 | `./pci_bar 6 0x0`、`./pci_bar 3 0x0`、`./pci_bar 5 0x0` | 示例程序允许 `bar_id<=6`；BAR3/BAR5 是 64 位 BAR 的高半（无独立 resource，`bar[3]/bar[5]` 仍为 NULL，`anlogic_pci_lib.c:947-1012`） | `bar[6]` 越界、`bar[3]/bar[5]=NULL` → `NULL/垃圾+off` 后 `ioread32` → 内核 NULL/野指针解引用 oops | **高**（仓库自带程序即可触发） |
| R7 | `cdev_ctrl.c:67-69`（`char_ctrl_read`） | `read()` 控制/用户节点 | 任何 read() | 源指针是 `gReadBuffer`，而本移植里它**从未分配**（`anlogic_pci_lib.c:134-141` 注释已说明；`dma_free_coherent` 传 NULL 属空转）→ `copy_to_user(buf, NULL, count)`（copy_to_user 有 fixup，不崩但恒 EFAULT），且函数仍 `return count` 谎报成功 | 中（功能错，非崩溃） |
| R8 | `anlogic_pci_lib.c:900-945`（`map_single_bar`） | `pci_iomap(dev, idx, map_len)`，map_len = BAR 长度（上限 INT_MAX） | probe 时对 BAR0..5 全部映射 | 映射本身不触设备（ioremap 懒映射），**安全**；但它使“窗内任意 offset”都变成合法内核虚拟地址 → 把 R1/R2 的越界后果从“立刻崩”变成“先静默打到 PCIe，再在 >1 MB 处崩”，是放大器而非根因 | 中（放大器） |
| R9 | `anlogic_pci_lib.c:112-126 + 152-166 + 1514/1522`（`XPCIe_ReadReg/WriteReg`/`XPCIe_InitCard`） | `readl/writel(gBaseVirt + 4*dw_offset)`，offset 0..9 | 当前调用点被注释（`:1522`），`gBaseVirt` 只在被注释的块里赋值（`:1514`）→ 现为 NULL | 一旦有人取消注释：① `gBaseVirt=NULL` → NULL+offset MMIO；② `XPCIe_InitCard` 会向 BAR0+0x0 写 DCSR 复位位、并向 +0x8..0x24 这些**本 EP 未译码**的 offset 写 → 复位用户逻辑、写进无意义地址 | 中（当前死代码，地雷） |
| R10 | `cdev_ctrl.c:219-261` 的 `bar_type` 字段 | 结构体里有 `bar_type` 但**被完全忽略** | — | 用户无法表达“我要 IO 空间还是 MEM 空间”，驱动也不校验 BAR 属性（`anlogic_pci_lib.c:1004-1010` 挑 user BAR 时才看 IORESOURCE_IO）→ 允许对 I/O BAR 做 MEM 语义访问，语义错但不崩 | 低 |
| R11 | `anlogic_pci_lib.c:1432-1448` | probe 时 `pcie_set_readrq(pdev, 256)`，注释写 “force MRRS to be 512” | — | 设置的是 **EP 自身**作为 requester 的读请求大小（影响 DMA 方向），不是 RC→EP 的 MRd 长度；注释与代码不符，会误导后续 DMA 测试的参数推断 | 低（文档/误导风险） |
| R12 | `anlogic_pci_lib.c:727-800`（`anlogic_isr`）、`cdev_ctrl.c:45-75` | 中断上下文：当前 `anlogic_isr()` 只 `printk` + 计数，**没有任何 MMIO** | — | 现状安全（这是好事：中断上下文里做 MMIO 一旦 CTO 就是 interrupt 上下文 oops → `die()` 里 `in_interrupt()` 直接 panic）。但 `printk` 在硬中断里每次中断都打，属噪声隐患 | 低 |
| R13 | `anlogic_pci_drv.c:178-198`（`anlogic_error_detected` 的 frozen 分支，:185-192） | AER 恢复线程上下文里 `pci_disable_device(pdev)` | 出现 frozen/fatal 错误时 | 该分支在 AER 工作线程里跑，`err.c` 侧还持 `device_lock`/总线锁；`pci_disable_device` 若与用户的 ioctl/ISR 竞争，可能把设备在访问中途关掉 → 后续 MMIO 全打到死设备 | 低-中（时序敏感） |

### 1.2 逐条说明“为什么会引发 RC 侧 Completion Timeout / AER / 内核卡死”

**(a) 内核态 MMIO 地址越界 —— 大多数情况下“根本到不了 EP”，先在 CPU 侧崩**
`map_single_bar()` 按 BAR 长度做 `pci_iomap`（`anlogic_pci_lib.c:929`，本板 BAR0=1 MB），所以 `xdev->bar[i] + off` 只在 `off < 0x100000` 时是合法内核映射。
`off ≥ 0x100000` 时 `ioread32` 访问的是未映射的内核虚拟地址 → `arch/arm64/mm/fault.c` 的 `do_translation_fault/do_page_fault` →
`die_kernel_fault("Unable to handle kernel paging request")` → `die()`。这类不会产生 Completon Timeout，而是**直接 oops**。

**(b) 真的打到 PCIe 且无人应答的场景（CTO 的来源）**
只有两种情况会让一次 MRd 产生“没有完成包”：
1. **链路 down 期间读 BAR 窗**（FPGA 热重配置/PERST/重训练时）。此时 CPU 侧地址完全合法（ioremap 有效），RC 的 iATU 窗也还在，
   MRd 出去后 EP 侧（正在重配置）不响应 → requester 侧 Completion Timeout。
   证据：DWC 自己的注释 `drivers/pci/controller/dwc/pcie-designware-host.c:729-738` 明说 “…platforms that forward errors on the system bus as
   **SError** upon PCI configuration transactions issued when the link is down”，而这段保护**只加在 config 路径**（`dw_pcie_other_conf_map_bus()` 里的
   `dw_pcie_link_up()` 检查），**MMIO 路径没有任何等价保护**（`ioread32` 是裸 AXI 访问，不经过内核）。
2. **物理地址在 RC 的 PCIe 窗口内、但不被 EP 的 BAR 认领**。本板 RC 的 memory 窗由设备树给出：
   `arch/arm64/boot/dts/rockchip/rk3588-extra.dtsi:377` → `<0x02000000 0x0 0xf0200000 ... 0x0 0x00e00000>`（**CPU 0xf0200000 起 14 MB**），
   内核按 host bridge window 逐个编程 iATU（`pcie-designware-host.c:866-909` `dw_pcie_iatu_setup()`），
   因此 **0xf0200000..0xf0ffffff 整段都会被转发到 PCIe 总线**，而不只是 EP 认领的那 1 MB。
   驱动侧（ioctl）永远在 BAR 资源内，所以踩不到；但 **mmap 路径（R3）可以让用户态访问到 0xf0300000 这类“窗内、BAR 外”的地址** → EP 不认领 → 完成包要么是 UR（较快，通常被合成全 F）要么根本不来（超时）。
   **UR 与超时的分界取决于 PH1P PCIe 硬核对未命中 BAR 的非投递请求是自动回 UR 还是静默丢弃 —— 这一点在本仓库没有证据，标为「未确认」（§5-1）。**
   旁证：厂商参考设计文档 `fpga/doc/APUG081_...docx` 正文明确写“所有 Memory BAR 共享同一块 ERAM…ERAM 实际大小为 4KB，如果读写地址超过 4KB，也会映射的 4KB 上”，
   说明**参考设计对 BAR 窗内所有地址都会应答**（不会 CTO）；本项目把这段 ERAM 换成了 `img/ep_bar_win.v`，其后缀解码只用 `[11:0]`（同上，仍全应答）。

**(c) 为什么这会变成“内核卡死”而不是无害的 0xffffffff**
- 同步外部中止（SEA）：`arch/arm64/mm/fault.c:865` `do_sea()` → 用户态 → 信号 `SIGBUS/BUS_OBJERR`（进程死，内核活）；
  **内核态 → `arm64_notify_die()`（`traps.c:291-303`）→ `die()`（`traps.c:206-232`）**：`in_interrupt()` 或 `panic_on_oops` 为真时 → `panic()`。
  本内核 `# CONFIG_PANIC_ON_OOPS is not set`（`.config:12059`）→ 普通 syscall 上下文里只是 oops+杀进程；但 oops 时若正持锁/在中断里 → panic 或后续死锁。
- 异步 SError：`traps.c:969 do_serror()` → `arm64_serror_panic()`（`traps.c:932`）→ `nmi_panic("Asynchronous SError Interrupt")` → **必定 panic**。
  RK3588 PCIe 的 AXI 错误究竟以“同步外部中止”还是“异步 SError”形式到 CPU，本机无法验证（未确认，§5-2）；但两者都足以解释“挂死/panic”。
- 另一条“看起来像卡死”的：AER 恢复里 `pci_bridge_wait_for_secondary_bus()` 最长 **60 s** 轮询 + 全程持总线/设备锁（§2.2）。

**(d) 与 FPGA 完成包长度相关的静默数据错（不是 CTO，但会误导判据）**
`pcie_ep_engine.v:586` 把完成包长度直接取请求长度（0 当 1024 dw），`:498/:502` 写进 CPLD 头，`:630-631/510` 对长读只把同一批 4 dw 反复移位发出去；
而 `img/ep_bar_win.v:108/238` 只准备 4 dw。结果是 **>4 dw（>16 B）的 MRd 会拿到一个“长度正确、数据重复/垃圾”的完成包**：
RC 不会报 CTO，但数据错，且 `code_review_pi_20261015.md` §H2 与 `..._run2.md:36` 已判定这属“静默数据错误”。
本次核对确认：厂商参考设计的自测项目本身就包含 `PRBS MEMRD BAR0 64 Bytes` / `BAR2 256/4096 Bytes`（APUG081 正文列表），
即**长读在参考设计里是常规用例**，换成只服务 4 dw 的 `ep_bar_win` 后这条用例会静默出错 —— 这也是“按 1 MB 全窗扫描”最可能得到错误结论的原因。

---

## 2 本内核的 AER 处理路径（文件 + 行号证据）

### 2.1 编译/开关现状（`third_party/linux/.config`）

| 配置 | 值 | 行号 |
|---|---|---|
| `CONFIG_PCIEPORTBUS` | `=y` | 1726 |
| `CONFIG_PCIEAER` | `=y` | 1728 |
| `CONFIG_PCIEAER_INJECT` | not set（无 aer_inject 注入通道） | 1729 |
| `CONFIG_PCIE_DPC` | not set（没有 DPC 下游端口遏制：错误不会被自动隔离） | 1737 |
| `CONFIG_PCI_ECAM` | `=y` | 1748 |
| `CONFIG_PCIE_ROCKCHIP_DW_HOST` | `=y`（RK3588 RC 用 DWC 驱动） | 1850 |
| `CONFIG_ARM64_EPAN` | `=y`（内核不能直接解引用用户地址） | 564 |
| `# CONFIG_PANIC_ON_OOPS` | not set → oops 不自动 panic | 12059 |
| `CONFIG_PANIC_TIMEOUT` | `=0` | 12060 |

**没有 `panic_on_oer` 这个开关**（grep 整个内核树无此符号）。可用的“AER 开关”只有：
- 启动参数 `pci=noaer`（`drivers/pci/pci.c:6757-6758` → `pci_no_aer()`，`pcie/aer.c:133`），
- `pcie_ports=native/compat`（`pcie/portdrv.c:614-630`），
- AER 服务本身还要求 MSI 可用：`pci_aer_available() = !pcie_aer_disable && pci_msi_enabled()`（`aer.c:138-141`），
- 逐设备的速率限制属性 `/sys/.../aer_dev_correctable`、`correctable_ratelimit_*`、`nonfatal_ratelimit_*`（`aer.c:644-725`；fatal 不限速，`aer.c:800-812`）。

### 2.2 事件路径（从 RP 中断到恢复动作）

```
RP 的 AER 中断
  → aer_irq()                       pcie/aer.c:1470        （硬中断，读 Root Error Status 塞 FIFO）
  → aer_isr()                       pcie/aer.c:1449        （线程化下半部）
  → aer_isr_one_error()             pcie/aer.c:1405
  → aer_isr_one_error_type()        pcie/aer.c:1373
  → find_source_device()            pcie/aer.c:1099        （先查 RP 自身，再遍历下级总线）
  → aer_process_err_devices()       pcie/aer.c:1352        （先全部打印，再处理，避免 reset 丢记录）
  → handle_error_source()           pcie/aer.c:1196
  → pci_aer_handle_error()          pcie/aer.c:1170
        ├─ AER_CORRECTABLE (aer.c:1175-1189)：清 COR_STATUS → 调驱动 cor_error_detected（若有）→ pcie_clear_device_status()
        │                                      **不做 recovery，不会 panic**
        ├─ AER_NONFATAL  (aer.c:1191)：pcie_do_recovery(dev, pci_channel_io_normal, aer_root_reset)
        └─ AER_FATAL     (aer.c:1193)：pcie_do_recovery(dev, pci_channel_io_frozen,  aer_root_reset)
```

AER 使能与中断打通（决定“这套机制在这块板上到底开没开”）：
- 每个枚举到的设备都会 `pci_aer_init()`（`drivers/pci/probe.c:2669` 调用；定义 `pcie/aer.c:385`），
  里面有 AER 能力时 `pci_enable_pcie_error_reporting()`（`aer.c:415-417`），后者置 `PCI_EXP_DEVCTL` 的 CERE/NFE/FE 位（`aer.c:253-262`）；
- AER 服务驱动 `aer_probe()`（`aer.c:1607`）只贴在 Root Port/RCEC 上，注册线程化 IRQ 后 `aer_enable_rootport()`（`aer.c:1533`）：
  清 Device Status、**关掉 SERR 上报**（清 `PCI_EXP_RTCTL` 的 `SYSTEM_ERROR_INTR_ON_MESG_MASK`）、清 Root Error Status，
  然后 `aer_enable_irq()`（`aer.c:1495-1503`）打开 `PCI_ERR_ROOT_COMMAND` 的中断；
- “内核是否原生拥有 AER”：DT 平台默认 `bridge->native_aer = 1`（`probe.c:671`），`pcie_aer_is_native()`（`aer.c:242-251`）；
- 于是 **错误消息不会变成 SERR/系统错误，而是走 AER 中断**：`aer_enable_rootport()` 明确关掉 SERR（`aer.c` 1538-1545）。

`pcie_do_recovery()`（`pcie/err.c:210-298`）的关键点：
- `frozen`（FATAL）**一定**做下属总线复位：`if (status == NEED_RESET || state == pci_channel_io_frozen)` → `reset_subordinates()`（`err.c:255-261`）；
- 驱动没有 `error_detected` 回调时投票 `NO_AER_DRIVER`（`err.c:63-81`）→ 后续回调被跳过、设备进入 disconnect；
- 我们驱动的投票：`anlogic_error_detected()`（`anlogic_pci_drv.c:178-198`）→ `io_normal → CAN_RECOVER`、`io_frozen → NEED_RESET`（并顺手 `pci_disable_device`）、
  `io_perm_failure → DISCONNECT`；`slot_reset`（`:200-215`）重新 `pci_enable_device_mem/pci_set_master/pci_restore_state`。
  注意我们**没有** `.mmio_enabled`，`resume` 里 `pci_cleanup_aer_uncorrect_error_status()` 被注释（`:217-221`）。
- 复位实体 `aer_root_reset()`（`aer.c:1667-1710`）→ `pci_bus_error_reset()`（`drivers/pci/pci.c:5686`）→ `pci_reset_bridge` →
  `pci_bridge_secondary_bus_reset()`（`pci.c:4854-4863`）→ `pci_reset_secondary_bus()`（`pci.c:4824-4842`，写 `PCI_BRIDGE_CONTROL.BUS_RESET`，msleep(2)）
  → `pci_bridge_wait_for_secondary_bus()`（`pci.c:4724-4823`）→ `pci_dev_wait(child, ..., PCIE_RESET_READY_POLL_MS - …)`
  （`pci.c:4793/4809/4821`），而 `PCIE_RESET_READY_POLL_MS = 60000`（`pci.c:82`）→ **最长约 60 s 的轮询等待**；
  期间 `pci_reset_bridge` 持 `pci_slot_mutex`，`report_error_detected()` 持 `device_lock`（`err.c:54,87`），其它 config/驱动回调会被挡住。
  复位超时后 `aer_root_reset` 返回 `DISCONNECT` → `err.c:262-264` 打 “subordinate device reset failed” → `err.c:296` “device recovery failed”。

### 2.3 “会不会把可纠正/不可纠正错误升级成 panic？”

- **不会。** `drivers/pci/pcie/aer.c` 全文件没有任何 `panic()`/`BUG()`（grep 结果为空），correctable 分支连 recovery 都不走（`aer.c:1175-1189`）。
- 也没有 `panic_on_oer`/`CONFIG_PANIC_ON_AER` 之类开关；AER 只做打印 + 标准恢复流程。
- `drivers/pci/pci-driver.c` 里没有 AER 的 panic 逻辑：AER 相关的只是一个条件编译标记（`pci-driver.c:1619`），
  设备移除走 `pci_device_remove()`（`pci-driver.c:498`）——它是正常的驱动解绑，不是 panic。

**但“不 panic”≠“不会挂死”**：见下节。

### 2.4 真正会把内核搞挂/搞卡的机制候选（按可能性排序，均为代码证据推导，非本机实测）

1. **异步 SError → 必然 panic**：`arch/arm64/kernel/traps.c:969 do_serror()` → `:932 arm64_serror_panic()` → `nmi_panic("Asynchronous SError Interrupt")`。
   诱因就是 §1.2(b)：链路 down 时的 PCIe 事务、或窗内无 BAR 认领的地址。DWC 源码注释（`pcie-designware-host.c:729-738`）承认该平台会把这类错误
   “forward errors on the system bus as SError”，而保护只覆盖 config 路径。
2. **内核态同步 SEA → oops**：`mm/fault.c:865 do_sea()` → `arm64_notify_die()`（`traps.c:291-303`）→ `die()`（`traps.c:206-232`）。
   因为 `PANIC_ON_OOPS` 未开，普通 ioctl 上下文里是 oops+杀进程（内核存活但已 taint）；若发生在中断/原子上下文 → `traps.c:226-227` 直接 panic；
   若发生在持锁路径，oops 之后不释放锁 → 表现为**后来者全卡死**。
3. **内核栈越界 → “Kernel stack overflow” panic**：`traps.c:926-928` 的 `nmi_panic(NULL, "kernel stack overflow")`。
   触发者就是 R2（`cdev_ctrl.c:271-272` 对局部 `rdata[]` 的越界写）。
4. **AER 恢复最长 60 s + 总线/设备锁**（§2.2 末）：不是 panic，但表现为“内核卡住不动”（`pci.c:82,4793,4809,4821`）。
   如果 FPGA 侧此时没回到链路 up，`pci_bridge_wait_for_secondary_bus()` 会一直等到超时，期间任何 config/ioctl 都在等锁。
5. **未确认候选**：RK3588 设备树给 PCIe 控制器列了 `interrupt-names = "sys","pmc","msg","legacy","err"`（`rk3588-extra.dtsi:361`），
   但内核 DWC 驱动**只请求 `legacy`**（`pcie-dw-rockchip.c:424`；EP 模式下请求 `sys`，`:701`）——**没有请求 `err`，也没有在驱动里屏蔽它**。
   若该“err”线硬件默认未屏蔽，一次 PCIe 错误（例如 AXI 解码错）可能造成中断风暴/“irq nobody cared”。
   本机无法确认其默认屏蔽状态 → 列入 §5-3 待验证。

### 2.5 为什么“热重配置后 BAR 全读回 0xffffffff”当时没把板子打死

配置空间读路径与 MMIO 路径不同：`pci_generic_config_read()`（`drivers/pci/access.c:88-105`）在 `map_bus()` 返回 NULL 时直接回 `PCIBIOS_DEVICE_NOT_FOUND`；
DWC 的 `dw_pcie_other_conf_map_bus()`（`pcie-designware-host.c:725-742`）在 `dw_pcie_link_up()` 为假时返回 NULL；
即使链路还在但 EP 不应答配置请求，硬件也会合成全 F —— `drivers/pci/pci.c:1220-1226` 的注释原文：
“…hardware generally synthesizes ~0 (PCI_ERROR_RESPONSE) data to complete the read.”
所以 **config 读回 0xffffffff 是“被设计成安全”的路径**，这解释了 `driver/README.md:55` 记录的现象。
**反过来，MMIO（mmap/resourceN 或 ioread32）没有这层保护**，这是本报告最需要传达的差别。

---

## 3 安全访问规约（后续 DMA 测试必须遵守的硬规则）

> 目标：把“任何一次 PCIe MMIO 读都必须确定性地拿到完成包、且内核态地址必须确定合法”变成可检查的规则。

**A. 只访问“已确认译码”的 offset（强制白名单）**
1. 只用 FPGA 寄存器表里登记的 offset：`0x40,0x44,0x48,0x4C,0x50,0x54,0x58,0x5C,0x60,0x64,0x68,0x6C,0x74,0x78,0x7C,0x80..0x9C`（`img/ep_bar_win.v:204-228` + `driver/README.md` 表）。
   其余 offset 在 FPGA 侧只会回 `DEADBEEF`（`ep_bar_win.v:228`），没有任何测试价值，只增加风险。
2. **禁止**对未登记 offset 做写入：写落到 `default` 无副作用，但一旦地址落在未译码区就是白打 TLP。
3. 不做“扫窗”式枚举（无论 1 MB 还是 14 MB）。要探测存在性就**读 0x40 MAGIC**（`ep_bar_win.v:204`，期望 `0x50434946` 经 bswap 规则换回后为 `0x46494350`，见 README 的字节序说明），不要靠遍历。

**B. 读前自检 + 读后熔断（MAGIC 门禁）**
4. 任何一次 MMIO 读之前：先读 `0x40`（MAGIC）。不等于期望值 → **立即停止**，不继续读任何寄存器、不写、不 DMA。
5. 任何一次读如果拿到 `0xffffffff`（或整段全 F）：视为“设备已不可用”，**立即停止本次测试并打印明确错误**；
   禁止把 `0xffffffff` 当作数据进行后续运算/写回/比较（这正是 `README.md:55` 记录状态的典型值：热重配置后全 F）。
6. 读写序列前记录“设备状态快照”（`lspci -vv` 的 `LnkSta`、`dmesg` 里 AER 计数），出错后先看快照再决定是否继续。

**C. 单次读长度与访问粒度**
7. 单次 MMIO 读 **≤ 4 dw（16 B）**，且只用 32 位对齐的 `ioread32`/`volatile u32*` 逐 dw 读；
   理由：`img/ep_bar_win.v:108/238` 只服务 4 dw，而 `pcie_ep_engine.v:586/498/502` 会按请求长度组完成包 → >16 B 静默错数据（§1.2(d)）。
8. 禁止在 BAR 上使用 `memcpy_fromio/toio`、`ioread32_rep`、`readq`、`ldp`/NEON 16 B 访问、以及任何“编译器/CPU 可能合并的宽访问”；
   README 已声明子字（<4 B）写语义未定义，只允许 32 位对齐访问。
9. 只对 `0x60`（DATA 窗口）做读，并遵守“读后 PTR 自动 +4、不保证同地址同值”的约定（`ep_bar_win.v:105/109`、`code_review_pi_20261015.md` §M3）；不要对 `0x60` 做预取式连续长读。

**D. 内核态/用户态的边界规则（对应 R1/R2/R3/R4）**
10. 驱动侧必须在 ioctl 入口**逐项校验**：`bar_id < XDMA_BAR_NUM(6)` 且 `xdev->bar[bar_id] != NULL`；
    `bar_offaddr + len <= pci_resource_len(pdev, bar_id)` 且 4 字节对齐；`len <= MAX_DATA_SIZE(100)`；
    违者返回 `-EINVAL`。（当前 `cdev_ctrl.c:219-280` 一项都没有。）
11. `BULK*` 的循环必须改成“0..len-1 的局部索引 + `bar_offaddr` 只用于地址”，并加长度上限：
    `for (i = 0; i < len && i < MAX_DATA_SIZE; i++) ... bar[bar_id] + bar_offaddr + (i<<2)`。
    现有写法 `addr+(i>>2)`（`cdev_ctrl.c:258/272`）本身也是地址算错的 bug（`bar_offaddr` 被计了两次）。
12. `mmap` 必须在算 `psize` 之前先 `if (off >= pci_resource_len(...)) return -EINVAL;`，并用有符号/溢出安全的方式比较
    （`cdev_ctrl.c:313-333`）；**禁止用无符号相减去表示剩余长度**。
13. `CFGW/CFGR` 必须改成 `copy_from_user/copy_to_user`；并且只允许访问合法 config offset（例如 `< 0x1000` 且按 4 对齐），
    绝不允许用户指定任意 `reg`。在这之前，**任何程序都不要调用这两个 ioctl**（会 oops）。
14. 用户态只用 `ioctl` 单 dw 读写，**不要 mmap resource0/resourceN**。若一定要 mmap，遵守：`offset=0`、`length ≤ 4 KB`、
    只按 4 B 访问、且每次访问前后校验 MAGIC。
15. 示例程序 `app/pci_bar.c:85` 的 `bar_id > 6` 必须改成 `>= 6`，并且**不要传 1/3/5**（I/O BAR 或 64 位 BAR 高半）。

**E. 中断与并发**
16. 中断上下文里不要做 MMIO（现在 `anlogic_isr()` 恰好没做，保持住）；MSI 尚未打通前一律用轮询，且轮询间隔 ≥ 1 ms。
17. 一次测试只允许一个进程持有设备（用 `flock`/自己加互斥）；AER 恢复期间（dmesg 出现 AER 打印）不得并发发起新 MMIO。

**F. 出错后的处置**
18. 一旦出现 AER 打印或 `dmesg` 里出现 `link down`/`Unsupported Request`/`Completion Timeout`：**停止**，
    记录 `dmesg`（AER 会打印 severity 与错误类型），然后**卸载驱动并重启主机再重新枚举**（`README.md:55` 已明确：不重启无法恢复 Gen2 x2）。
19. 需要降低“错误即恢复动作”的干扰时，可用 `pci=noaer` 启动（`pci.c:6757`）做隔离对比 —— 但注意这**只是不处理 AER**，MMIO 的
    CTO/SError 依然会打到 CPU，不能当作“安全开关”。

---

## 4 如果要 FPGA 侧加固：应验证的点与验证方法

> 原则：**“窗内所有地址都必回完成包”优先于“回固定值”。** 只要有一次 MRd 拿不到完成包，主机侧就存在 CTO → AER/SError 的可能。
> 以下每一项都给出“验证什么 / 怎么验证 / 期望”。凡需要上板或需要仿真器的，本报告**未执行**（见 §5）。

**V1. BAR 命中/未命中的应答语义（最高优先）**
- 验证：BAR 窗内地址（含未登记 offset、含 4 KB 别名区）**都必须有完成包**；BAR 窗**外**（例如 EP 只认领 1 MB，而访问 0xf0300000，仍在 RC 的 14 MB 窗内）
  的 MRd，PH1P 硬核到底是“自动 UR 完成（主机读回全 F）”“返回 CPLD 状态=UR”还是“静默丢弃（→ CTO）”。
- 方法（不需要改代码）：① 读厂商硬核文档/参考设计（`fpga/doc/APUG081_...docx` 只说明了“Memory BAR 共享 4 KB ERAM、超 4 KB 映射回 4 KB”，
  **没有说明未命中 BAR 的行为** → 需向安路确认或做实验）；② 主机侧最小实验：加 `pci=noaer`，用一个**只读单 dw、带超时保护**的用户态程序
  依次读 `0x40`（BAR 内）与 BAR 窗后 1 MB 处（窗内 BAR 外），观察返回值与是否返回；③ 之后立即 `dmesg | grep -i aer` 查看 RP 是否记录了
  Unsupported Request / Completion Timeout（AER 计数 `aer_dev_correctable/uncorrectable` 在 `aer.c:581-601` 注册，可直接读 sysfs）。
- 期望：BAR 外一律 UR（可预测、主机读全 F、不 hang），绝不静默丢弃。

**V2. 完成包长度必须按“实际服务的 dw 数”夹紧**
- 验证：请求 1/2/3/4/5/8/16/32 dw 时，CPLD 的 length/byte count 与**实际返回的载荷**一致；>4 dw 时必须截断为 4 dw 或回 UR。
- 方法：TLP 级测试台或波形抓取（关注 `pcie_ep_engine.v:586` 的 `tl_mr_len_ff`、`:498/:502` 的 length/byte count、`:630-631` 的 cycle 数），
  再在主机侧用“读 N dw 后比对数据与期望”做端到端确认。注意本机**没有 iverilog/verilator/vsim**（`code_review_pi_20261015.md` §2.3-M7 已记录），
  所以这一步目前只能分析、不能仿真。

**V3. 4DW（64 位 BAR）地址解码**（已知缺陷 `code_review_pi_20261015.md` §H1）
- 验证：BAR2/BAR4 的写/读是否真正落到目标寄存器（现在 `ep_bar_win.v:98` 只取 `rx_data_ff[191:160]`，4DW 时应拼 `[191:160],[159:128]`，
  于是 64 位 BAR 访问的 `tlp_addr[11:0]=0` → 读回 `DEADBEEF` 或错寄存器）。
- 方法：对 BAR2/BAR4 各写一个 SCRATCH 语义值（若支持）再读回；或先用 `lspci -vv` 确认 BAR2 是 64 位（`td_20261015_123428.log:158` 的
  `PCIE_CFG_PF0_BAR2_64BIT=1'b1`），再读 `0x40` 期望 MAGIC。

**V4. 12 位地址别名与窗内全覆盖**
- 验证：`ep_bar_win.v:105/146/204` 只比较 `[11:0]` → 1 MB 窗内每 4 KB 出现别名（256 份）。当前这**恰好保证了“窗内必答”**，
  但会让“访问了未映射地址”失去保护。
- 方法：读 `BAR+0x1060` 与 `BAR+0x60`，确认两者一致（别名成立）→ 记录为已知行为；若要加固，按实际 BAR 尺寸做全位比较并让未命中回 `DEADBEEF`（仍然要有完成包）。

**V5. 写被忽略地址的行为**
- 验证：对未登记 offset 的写（MWr，非投递）是否被静默丢弃；是否会在 RP 侧记 Unsupported Request。
- 方法：主机侧写一个无副作用 offset，然后 `dmesg | grep -i aer` + 读 RP 的 AER 计数；本机无法执行。

**V6. 链路 down / 热重配置期间的行为**
- 验证：① 重配置后 BAR0 解码是否自动恢复（还是必须重新枚举，实测记录 `README.md:55` 说要重启 RC）；
  ② 重配置瞬间主机侧的 AER/LKSTA 记录形态；③ EP 侧 `tl_rst_user` 复位后 `ep_bar_win` 的寄存器（`ep_bar_win.v:123`）与相机块状态是否对称
  （`code_review_pi_20261015.md` §2.2-H6 已指出热复位后 tap 被清回默认但相机块不重启）。
- 方法（只读）：重配置前后各读一次 RP 的 `LnkSta`（`lspci -vv`）与 `0x40/0x9C`，比较；不要在此期间发起 MMIO 风暴。
  **本次未执行（禁止上板）。**

**V7. 中断能力现状（影响 DMA 测试方案）**
- 验证：EP 没有 MSI/MSI-X 能力（`README.md:53`，capability 链只有 Express/PM/Vendor/ARI/AER）→ 后续 DMA 完成只能轮询或走 FPGA 侧补 MSI。
- 方法：`lspci -vv` 看 Capabilities 列表；不要按“有中断”设计 DMA 流程。

---

## 5 未确认 / 无法验证的清单（严格区分）

1. **PH1P 硬核对“未命中 BAR 的非投递请求”是回 UR 还是静默丢弃 —— 未确认。** 仓库内（RTL、APUG081 文档、code_review 报告）都没有证据；
   这决定了“窗内 BAR 外读”是 all-F（安全）还是 CTO（危险）。
2. **RK3588 把 PCIe 控制器的 AXI 错误上报成“同步 SEA”还是“异步 SError” —— 未确认。**
   内核里两条路径的后果不同（oops vs 必然 panic），需要上板看真实 dmesg 才能定。
3. **设备树里的 PCIe `err` 中断是否默认被硬核屏蔽 —— 未确认。** 驱动未请求该中断（`pcie-dw-rockchip.c:424` 只请求 `legacy`），
   若未屏蔽则可能中斷风暴。需要只读地查 `/proc/interrupts` 与 `PCIE_CLIENT_INTR_MASK_*` 的复位默认值。
4. **AER 服务是否真的绑到了本板 RP —— 未确认。** 依据代码（`CONFIG_PCIEAER=y` + DT 平台默认 `native_aer=1`）推断为“是”，
   但需要上板用只读命令确认：`ls /sys/bus/pci/devices/0000:00:00.0/ | grep aer`、`dmesg | grep -i "AER.*enabled"`。
5. **>16 字节 MRd 在实机上是否真的发生过 —— 未确认**（`code_review_pi_20261015_run2.md:145` 已明确写过这一条）。
   驱动侧代码证据只指向单 dw 读（`cdev_ctrl.c:241/272`），但 `resource0` mmap 后的用户态 `memcpy` 是否产生更长 MRd 从未观测过。
6. **本次板卡 panic 的具体归因 —— 未确认。** 板子 dmesg/panic 原文不在仓库（已搜索工程目录与 `/tmp`，未见），
   所以 §2.4 是“候选机制排序”，不是“事故结论”。
7. **BAR2 在 CPU 侧的物理基址**：本板 DT 的 64 位窗在 `0x9_00000000`（`rk3588-extra.dtsi:378`），
   而 `PORTING.md` 说驱动选中的 user BAR 是 BAR2 —— 其实际 CPU 基址需要 `cat /sys/bus/pci/devices/.../resource` 确认（未执行）。
8. **未运行**：任何构建、insmod、板卡命令、仿真（本机无 iverilog/verilator/vsim）、联网检索。

---

## 6 证据索引（本次实际读到的文件）

**驱动/用户态**
- `anlogic_pcie_drv/cdev_ctrl.c`：45-75（read）、139-153（cfg 读写）、219-247（IOCW/IOCR）、248-280（BULKW/BULKR）、281-289（CFGW/CFGR）、298-350（bridge_mmap）
- `anlogic_pcie_drv/anlogic_pci_lib.c`：92/112-126（gBaseVirt、XPCIe_Read/WriteReg）、129-144（XPCIe_exit）、152-167（XPCIe_InitCard）、727-800（`anlogic_isr`）、851-880（set_dma_mask）、
  900-945（map_single_bar/pci_iomap:929）、947-1012（map_bars/user bar 选择）、1056-1062（request_regions）、1175-1215（engine_init:1196）、
  1236-1290（probe_for_engine）、1400-1448（device_open: pci_enable_device 1425、pcie_set_readrq 1441）、1505-1526（被注释的 gBaseVirt/InitCard 调用）
- `anlogic_pcie_drv/anlogic_pci_lib.h`：125（XDMA_BAR_NUM=6）、600-641（struct anlogic_dev，`bar[XDMA_BAR_NUM]` 在 614）
- `anlogic_pcie_drv/cdev_ctrl.h`：82-98（ioc_bar/bulk 结构、MAX_DATA_SIZE=100）、120-121（BULKW/BULKR 命令码）
- `anlogic_pcie_drv/anlogic_pci_drv.c`：102-107（read/write_register）、178-227（err_handler 各回调）、252-260（pci_error_handlers）
- `anlogic_pcie_drv/anlogic_pci_cdev.c`：133-160（xcdev_check）、270-345（create_xcdev）、612（user cdev 创建）
- `anlogic_pcie_drv/cdev_events.c`：24-119（events 节点；本板未创建）
- `app/pci_bar.c`：33-58（reg_read/write 走 ANLOGIC_IOCR/IOCW）、74-77（bar_id<=6）、85-86（off-by-one 检查）、92-94（4 字节对齐检查）
- `driver/README.md`：53-58（已知问题：0xffffffff、必须重启、子字写未定义、BAR 共享/别名）、寄存器表
- `anlogic_pcie_drv/PORTING.md`：移植改动与 2026-10-15 实测（PIO 0x12345678 回读、BAR2 作 user bar、legacy 中断）

**FPGA 侧（只读，用于判定“EP 是否应答/答什么”）**
- `fpga/rtl/ep_side/pcie_ep_engine.v`：211-246（`memrd_flag/memwr_flag` 仅按 TLP 类型判定，**不看 BAR 命中**）、
  370-390（TX FSM 入口）、469-513（CPLD 组包）、580-589（`tl_mr_len_ff`=请求长度，0→1024）、621-642（长读用重复 4 dw 移位）、
  154-155（`tl_rx_wait=0`、`tl_rx_masknp=0`）
- `fpga/rtl/ep_side/img/ep_bar_win.v`：94-109（3DW/4DW、仅 `[11:0]` 比较、只服务 4 dw）、195-240（寄存器读 mux、`default: DEADBEEF`、载荷布局）
- `fpga/doc/code_review_pi_20261015.md`：§2.2-H1/H2/H4/H6、§2.3-M3/M4/M7
- `fpga/doc/code_review_pi_20261015_run2.md`：§二 H3、:36（长读后果矩阵）、:145（>16B MRd 未实测）
- `fpga/doc/APUG081_...docx`（正文经 `unzip -p` 管道只读提取）：EP BAR 配置表（BAR0 1 MB、BAR2/3 与 BAR4/5 是 64 位 BAR）、
  “所有 Memory BAR 共享 4 KB ERAM，超 4 KB 地址映射回 4 KB”、自测含 `PRBS MEMRD BAR0 64 Bytes`/`BAR2 4096 Bytes`

**内核（路径前缀 `~/Projects/orangepi-5-max-bsp/third_party/linux/`）**
- `.config`：512、564、1726-1748、1792-1851、12059-12063
- `drivers/pci/pcie/aer.c`：133-141、242-262、385-417、581-601、644-725、800-812、1099、1170-1196、1247-1250、1352、1373-1440、1449-1503、1533-1545、1607-1640、1667-1710
- `drivers/pci/pcie/err.c`：49-96、210-298
- `drivers/pci/pcie/portdrv.c`：614-630
- `drivers/pci/probe.c`：669-673、2669
- `drivers/pci/pci.c`：74、82、1195-1240、4709-4863、5683-5690、6757-6758
- `drivers/pci/access.c`：88-126
- `drivers/pci/controller/dwc/pcie-designware-host.c`：725-742（SError 注释 + link_up 检查）、768-826（conf ops）、866-909（iATU 按 host bridge window 编程）
- `drivers/pci/controller/dwc/pcie-designware.c`：533-597（iATU 编程与 ETIMEDOUT）、946-1000（iATU 探测，含 0xFFFFFFFF 判定）
- `drivers/pci/controller/dwc/pcie-dw-rockchip.c`：421-446（host init）、617-623（ops）、644-670（EP 侧 sys irq；RC 不用）、870 行全文 grep `aer` 无命中
- `arch/arm64/mm/fault.c`：341-355（die_kernel_fault）、638-667（uaccess 规则）、865-891（do_sea）、930（fault_info “synchronous external abort”）
- `arch/arm64/kernel/traps.c`：176-232（__die/die）、291-303（arm64_notify_die）、920-975（stack overflow / arm64_serror_panic / do_serror）
- `arch/arm64/boot/dts/rockchip/rk3588-extra.dtsi`：355-385（pcie3x4：`ranges` 内存窗 0xf0200000+0x00e00000、`interrupt-names` 含 `err`）

---

## 7 一句话给下一步

在改任何一行驱动/用户态代码之前，**先把 §3 的 A/B/C/D 四项落到工具与驱动校验里**（尤其是“MAGIC 门禁 + 全 F 熔断 + 单次 ≤4 dw + mmap/bar_id 校验”），
再去碰 DMA；否则任何一次“窗扫描”“长读”“越界 offset”都会把这次的事故形态原样复现。
FPGA 侧的加固顺序建议是 **V1（必答）→ V2（长度夹紧）→ V3（4DW 解码）→ V4（别名与未命中回值）**。
