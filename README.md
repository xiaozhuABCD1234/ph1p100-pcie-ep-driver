# PH1P100 PCIe EP —— RK3588 主机侧驱动

安路 PH1P100 作 PCIe 端点（EP），RK3588 作 Root Complex。
本仓包含内核模块与 Rust 用户态工具：BAR 访问、一致性收帧缓冲、链路/推帧控制、
RKA1 one-shot 授权，以及取帧、状态查询和 R10F 严格校验。

## 目录

| 路径 | 内容 |
|---|---|
| `src/`、`include/`、`Makefile` | 内核模块；`include/anlogic_pci_ioctl.h` 是驱动/用户态唯一的 ioctl ABI |
| `app/rs/` | Rust 工具，六个 bin：`pcie_check`、`pci_bar`、`link_ctl`、`frame_grab`、`r10f_check`、`cam_grab` |
| `docs/protocol/` | RKA1/R10F 冻结合同、合成向量、严格校验器 |
| `docs/ENGINEERING.md` | 工程边界、脚本准入与合并门槛 |
| `docs/changes.md` | 4K 参数化改动记录 |

寄存器大表、缓冲布局与诊断位域统一见
[../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)；本文只写驱动接口与使用。

## 构建与部署

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

板上加载（rootfs 里可能已有旧 ko，先覆盖再 depmod）：

```sh
cp anlogic_pci.ko /usr/lib/modules/$(uname -r)/updates/
depmod -a
rmmod anlogic_pci 2>/dev/null || true
modprobe anlogic_pci
ls /dev/ANLOGIC-PCI0_*
```

当前 1920×1088 灰度 JPEG 位流用：

```sh
insmod ./anlogic_pci.ko frame_width=1920 frame_height=1088 frame_slots=1
```

1080p 双路与 4K 采集位流的参数见 `docs/changes.md`。

## 设备节点与接口

| 节点 | 用途 |
|---|---|
| `/dev/ANLOGIC-PCI0_control` | 寄存器/BAR ioctl、链路状态、RKA1；也可发 FRAMEINFO/FRAMEWAIT |
| `/dev/ANLOGIC-PCI0_user` | 用户 BAR 只读/只写窗口（mmap 一页） |
| `/dev/ANLOGIC-PCI0_frame` | 收帧缓冲：只读 mmap + `FRAMEINFO`/`FRAMEWAIT` |

主要 ioctl（编号与结构体以 `include/anlogic_pci_ioctl.h` 为准）：

- `ANLOGIC_IOCFRAMEINFO`：物理地址、槽步长、槽长、几何、seq/count、mailbox 偏移；
- `ANLOGIC_IOCFRAMEWAIT`：轮询等新帧，`timeout_ms=0` 只查一次，上限 600 s；
- `ANLOGIC_IOCLINKSTATUS` / `LINKRETRIG` / `PUSHCFG`：链路、重触发、推帧地址复核；
- `ANLOGIC_IOCRKA1`：`PROBE/CONFIGURE/ARM/ACK/CANCEL`，one-shot 推帧必须走这里。

收帧缓冲由驱动用 `dma_alloc_coherent` 分配；槽 0 的 128 B 之后是 R10F 数据起点，
mailbox 在槽区之后独立一页。FPGA 每推完一帧写门铃，驱动补槽头与 `mailbox.seq`。
取帧端只读 mmap，不要按 1920/1080 猜偏移，一律用 ioctl 报出的值。

## 常用命令

```sh
./pcie_check                 # 枚举/链路/BAR/节点自检
./pci_bar 0 0x40             # 应读回 0x50434946
./link_ctl status            # 链路、DDR3、推帧与 mailbox
./link_ctl arm -r A          # 申请一帧额度
./frame_grab -n -t 4000 -o dma_frame.pgm
./link_ctl ack               # 释放 HELD
./r10f_check frame.dma       # 离线严格校验
```

双路：先 `link_ctl sync start -t 5000`，再用 `frame_grab --pair` 或
`display-pair --source device-pair`。

## 模块参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `frame_width` / `frame_height` | 1920 / 1080 | 收帧几何，必须与位流一致 |
| `frame_stride` | 0 | 行 packed RAW10 字节数；0 = `width/4*5`，非 0 必须相等 |
| `frame_slot_bytes` | 0 | R10F 槽长；0 = 自动推导，非 0 只做交叉校验 |
| `frame_slots` | 2 | 1 = 单路，2 = A/B 双路 |
| `use_irq` | 0 | 当前 RTL 不发 MSI，收帧走 mailbox 轮询 |
| `interrupt_mode` | 1 | 仅 `use_irq=1` 时有效：0=MSI-X，1=MSI，2=Legacy |

## 已知限制

- 当前 JPEG 位流是 1920×1088 单路；4K 几何/分配已实现但未在板实收。
- A 路已实测；B 路代码路径存在，本轮未做板级实收。
- MSI 中断路径未验证；`lane_valid` 是瞬时采样，以 `0x4C frame_cnt` 是否增长为准。
- RKA1 是 fail-closed：旧位流没有 `RKA1` 扩展时推帧不可用，采集/诊断仍可用。
- FPGA 位流是 SRAM 配置，重下/掉电后必须重启 RK3588 才能重新枚举 Gen2 ×2。
- R10F 头 0x28 的 bayer_phase 是 RTL 占位值；当前安装 A/B 均按 RGGB，换模组须重测。

## 合并前检查

```sh
make -C <内核树> M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
cd app/rs && cargo build --release --target aarch64-unknown-linux-gnu
python3 docs/protocol/rka1-r10f-v1/tools/r10f_strict.py selftest
python3 docs/protocol/rka1-r10f-v1/contract/check_contract.py
git diff --check
```
