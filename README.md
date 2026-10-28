# PH1P100 PCIe EP —— RK3588 主机侧驱动

安路 PH1P100 作 PCIe 端点，RK3588 作 RC。内核模块负责 BAR 访问、一致性收帧缓冲、
链路/推帧控制与 RKA1 one-shot；用户态 Rust 工具负责链路自检、取帧、ARM/ACK 和 R10F 严格校验。

本文件只写已经实现并在板子上验证过的内容；未验证项会明确标出。

## 已验证状态（2026-10-28，RK3588 板）

| 项 | 结果 |
|---|---|
| 链路 | `0000:01:00.0`，`1edb:abcd`，PCIe Gen2 ×2 |
| 内核模块 | 本仓源码编译通过，`rmmod` / `insmod` 正常，probe 无报错 |
| 设备节点 | `_control`、`_user`、`_frame` 三个齐全 |
| BAR0 | 1 MiB，+0x40 读回 `0x50434946` |
| 收帧 | 1920×1080 R10F，2,592,768 B/帧，三层 CRC 通过 |
| 取帧 | `frame_grab` 输出 1920×1080 PGM，2,073,600 B 像素数据 |
| ARM/ACK | `link_ctl arm -r A` → 推帧 → `link_ctl ack` 完整通过 |

未在本轮验证：B 路实收、4K 几何、MSI 中断、100 帧/10 min 长稳。

## 目录

| 路径 | 内容 |
|---|---|
| `src/`、`include/`、`Makefile` | 内核模块。`include/anlogic_pci_ioctl.h` 是驱动与用户态唯一的 ioctl ABI |
| `app/rs/` | Rust 用户态（`anlogic_pci_tools`），六个 bin：`pcie_check`、`pci_bar`、`link_ctl`、`frame_grab`、`r10f_check`、`cam_grab` |
| `docs/protocol/` | RKA1/R10F 冻结合同、合成向量、严格校验器 |
| `handoff/` | A1U1 文件样例与解码包 |
| `experiments/` | 本地一次性实验，不提交 |
| `docs/ENGINEERING.md` | 工程边界、脚本准入与合并门槛 |

## 构建

内核模块（内核树外构建，产物 `anlogic_pci.ko` 落在仓库根）：

```sh
make -C ~/Projects/orangepi-5-max-bsp/third_party/linux \
     M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
```

用户态（aarch64 静态链接）：

```sh
cd app/rs
cargo build --release --target aarch64-unknown-linux-gnu
```

## 加载

rootfs 的 `/usr/lib/modules/$(uname -r)/updates/` 下可能已有旧版 `anlogic_pci.ko`。
要确保加载的是本仓版本，覆盖该路径后执行 `depmod`：

```sh
cp anlogic_pci.ko /usr/lib/modules/$(uname -r)/updates/
depmod -a
rmmod anlogic_pci 2>/dev/null || true
modprobe anlogic_pci
ls /dev/ANLOGIC-PCI0_*
```

默认 `use_irq=0`，收帧走 mailbox 轮询。只有 FPGA 确认会发 MSI 时才考虑
`use_irq=1 interrupt_mode=1`（本版未在板上验证中断路径）。

模块参数：

- `frame_width` / `frame_height`：默认 1920×1080；
- `frame_slot_bytes`：0 = 按几何推导 R10F 长度；
- `frame_slots`：只支持 1，推帧目标固定为槽 0。

## 使用

```sh
# 链路/枚举/BAR/节点自检，只读 sysfs
./pcie_check

# BAR0 + 0x40：应读回 0x50434946
./pci_bar 0 0x40

# 查链路、DDR3、推帧与 mailbox 状态
./link_ctl status

# 申请一个 one-shot 额度（A 路）
./link_ctl arm -r A

# 等新帧并写 PGM
./frame_grab -n -t 4000 -o dma_frame.pgm

# 释放 HELD，CTRL 收回 0x21
./link_ctl ack

# 对一份 R10F 文件做严格校验
./r10f_check frame.dma
```

`cam_grab` 是 T07 之前的 640×480 PIO 路径，当前位流默认拒绝，只有调试老位流时用 `--legacy`。

## 收帧缓冲布局

驱动在 probe 时用 `dma_alloc_coherent` 分配一块一致性内存：

- 每帧 R10F 传输长度：2,592,768 B = 128 B 头 + 2,592,000 B packed RAW10 + 640 B 尾零；
- 槽步长：2,596,864 B（4 KiB 对齐）；
- 槽内数据偏移：128 B；
- 缓冲末尾一页是 mailbox，位置由 ioctl 报出的 `mb_off` 决定，不要按常量猜；
- FPGA 的 MWr 目标是槽 0 数据起点，门铃写 mailbox 的 `push_dbell`。
- 当前安装 A/B 两路均按 **RGGB**（同场景告示牌在 A/B 上都还原成黄色；早期 BGGR 来自翻转预览/另一套安装）。头 0x28 当前写 3(GBRG)，是 RTL 占位推导，不能直接拿去做去马赛克；换模组/改 ROI 后必须用已知色卡重测。

R10F 头、payload、尾零和三段 CRC 的判据见 `include/anlogic_pci_ioctl.h` 与
`docs/protocol/rka1-r10f-v1/tools/r10f_strict.py`；Rust 侧 `r10f_check` 使用同一套规则。

## RKA1 one-shot 与 CTRL

- 每帧需要一次 ARM；推完一帧进入 HELD，ACK 匹配后才释放下一帧额度。
- 驱动 probe 会自动 ARM 一次，所以上电后通常已经有一帧可读。
- 用户态用 `link_ctl arm` / `link_ctl ack` 管理额度；`frame_grab` 只负责取帧。
- `CTRL[2]` 是传感器重触发电平：电平变化会打断写者，任何 CTRL 写入都必须保持 bit2 不变。
  `link_ctl arm/ack` 在写前后做 bit1/bit2 门禁和整字回读复核。
- 常见 `0xC8`：`5` = 有完整帧但没有 ARM 额度；`8` = HELD 期间源又出帧（已忽略）。
  两者都不是数据通路故障。

## 已知边界

- `link_ctl status` 在 ACK 后 `push_en=0` 时会按“推帧未初始化”返回 4；这是 one-shot 空闲态。
- `lane_valid` 是瞬时采样，单次读到 0 不等于链路丢失；以 `0x4C frame_cnt` 是否增长为准。
- A 路已在本板实测；B 路代码路径存在（`arm -r B`），但本轮未做板级实收。
- 4K 几何、MSI 中断、长时间稳定性不在本轮验证范围。
- FPGA 位流是 SRAM 配置；重新下载或掉电后必须重启 RK3588 才能重新枚举 Gen2 ×2。

## 合并前检查

```sh
make -C <内核树> M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
cd app/rs && cargo build --release --target aarch64-unknown-linux-gnu
python3 docs/protocol/rka1-r10f-v1/tools/r10f_strict.py selftest
python3 docs/protocol/rka1-r10f-v1/contract/check_contract.py
git diff --check
```
