# driver 4K（3840×2160）支持改动说明

本文只列本次 4K 相关改动；`driver/` 工作区里已有的双路 / sync 等未提交改动不在本文范围。
本次未修改 `display/`、`fpga/`，未执行 git commit / push / checkout。
行号按写本文时的 `nl -ba` 结果，后续改动可能使其偏移。

## 1. 几何与契约

固定契约按 2026-10-29 更正后的数字：

| 项 | 1080p | 4K |
|---|---|---|
| 分辨率 | 1920×1080 | 3840×2160 |
| packed RAW10 行 stride | 2400 B | 4800 B |
| payload | 2,592,000 B | 10,368,000 B |
| R10F 槽长（128B 头 + payload + 4KiB 对齐尾） | 2,592,768 B（633×4KiB） | 10,371,072 B（2532×4KiB） |
| 尾零 | 640 B | 2,944 B |
| 槽步长 PAGE_ALIGN(128 + 槽长) | 2,596,864 B | 10,375,168 B |
| 128B MWr 笔数（槽长/128） | 20,256 | 81,024 |

4K 单路两槽 + mailbox 页的分配量：`2×10,375,168 + 4,096 = 20,754,432 B`。

原先口头的 10,370,112 B 不是 4 KiB 整数倍（`10,370,112 mod 4096 = 3136`），
与“4 KiB 对齐尾”自相矛盾；按 `ceil((128+10,368,000)/4096)×4096` 应为 10,371,072 B。

## 2. 改动清单

| 文件:行 | 改动 | 原因 |
|---|---|---|
| `include/anlogic_pci_ioctl.h:121-135` | 保留 1920×1080 默认/参考常量，补 4K 推导值说明；不改结构体布局与 ioctl 编号 | 默认值必须保持 1080p，同时让后续读者不再把 1080p 常量当通用值 |
| `include/anlogic_pci_ioctl.h:224,375` | 0xCC / `push_tlp_cnt` 注释从固定“正常 2400”改为 `slot_bytes/128`（1080p 20,256，4K 81,024） | 原注释与实际 RTL 的 128B MWr 约定不符，4K 下更不能写死 |
| `src/anlogic_pci_frame.c:8-12` | 分配注释改为 2 槽默认与 4K 推导公式 | 注释与新的按时计算一致，避免再按 1080p 单槽理解 |
| `src/anlogic_pci_frame.c:66-84` | 新增 `frame_stride` 模块参数（行 stride，0=width/4*5）；`frame_slot_bytes` 改为可选交叉校验 | 让槽长完全由 width/height/stride 推导；非 0 的 stride 必须等于 width*5/4 |
| `src/anlogic_pci_frame.c:172-195` | `anlogic_pci_frame_r10f_bytes()` 增加 `line_stride` 入参；校验 width 为 4 的倍数、line_stride==width/4*5、帧长上限 | 4K 槽长 = align4k(128 + 4800×2160)，不再依赖 1080p 常量 |
| `src/anlogic_pci_frame.c:204-251` | `frame_alloc` 用 width/height/line_stride 现算 `derived_slot`；槽步长/分配量继续按时算 | 确认 `dma_alloc_coherent` 的大小不是常量：4K 两槽 + mailbox 为 20,754,432 B |
| `src/anlogic_pci_frame.c:324-327` | 启动日志增加“行 stride”字段 | 现场能从 dmesg 直接核对 4800 / 10,371,072 |
| `src/anlogic_pci_rka1.c:149-166` | 容量下限只用 `hw->frame_slot_bytes`；为 0 时 `-ENODEV`，不再回退 1080p 常量 | 避免 4K 时用 2,592,768 的错误下限授权 |
| `src/anlogic_pci.h:130-131` | 注释区分内部槽步长 `frame_stride` 与模块参数行 stride | 两者同名但语义不同，防止后续改错 |
| `app/rs/src/ioctl.rs:109-112,328-329` | `FrameMailbox` 补 `push_dbell_b`（+36）与 6 个 reserved，按头文件钉死偏移 | 与并发双路改动后的 mailbox ABI 对齐；不影响原有 32/64 字节布局 |
| `app/rs/src/bin/frame_grab.rs:254-266` | `descriptor_check` 增加槽长与几何推导值的一致性检查 | 4K 槽长必须是 10,371,072；驱动/工具/FPGA 任一方给错立即拒绝 |
| `app/rs/src/bin/frame_grab.rs:314,795-797` | 行 stride/槽内容注释改为 1080p / 4K 两个值 | 4K PGM 预览是 3840×2160×1B = 8,294,400 B，不等于槽长 |
| `app/rs/src/bin/frame_grab.rs:1106-1133` | 增加 1080p/4K 描述符自检单测（含 10,370,112 反例） | 把两种几何的 slot/stride/分配量钉住 |
| `app/rs/src/bin/link_ctl.rs:394-418` | `status` 按 `0xAC/128` 推导期望 MWr 数并对照 0xCC | 把“2400 笔”的旧说法换成由帧长现算，4K 期望 81,024 |
| `app/rs/src/bin/r10f_check.rs:15-16` | 用法补 4K 示例 | 4K 校验时显式给 `--width 3840 --height 2160` |
| `README.md:66-90,128-131` | 模块参数、4K/1080p 加载命令、4K 布局与分配量 | 现场按文档即可选 1080p 默认或 4K 参数启动 |

## 3. 关键代码路径

1. probe → `anlogic_pci_frame_alloc()`：
   ```
   row_bytes   = frame_width / 4 * 5          // 4K = 4800
   payload     = row_bytes * frame_height     // 4K = 10,368,000
   slot_bytes  = align4k(128 + payload)       // 4K = 10,371,072
   stride      = PAGE_ALIGN(128 + slot_bytes) // 4K/4KiB 页 = 10,375,168
   size        = stride * slots + PAGE_SIZE   // 4K 两槽 = 20,754,432
   ```
   `frame_slot_bytes` 非 0 时只与 `slot_bytes` 比较；不一致直接 `-EINVAL`。
   分配失败（例如 CMA 不足）时 probe 失败，不会带着 1080p 槽继续跑。
2. `ANLOGIC_IOCFRAMEINFO` 原样报出 `width/height/stride/slot_bytes/hdr_bytes/slots/mb_off`，
   用户态不按 1920/1080 猜。
3. R10F 校验（Rust `r10f::parse`）按头里的 `width/height/stride` 与
   `payload_bytes == stride*height`、`dma_bytes = align4k(128+payload)` 校验，4K 与 1080p 同一套代码。
4. `frame_grab` 拷贝 `slot_bytes`，用 R10F 头里的行 stride 解 packed RAW10；
   4K 输出 P5 头 `P5\n3840 2160\n255\n`，像素 8,294,400 B。
5. `link_ctl status` 的 0xCC 自检：`expect_tlp = ddr_len / 128`（要求 `ddr_len % 128 == 0`），
   1080p 应为 20,256，4K 应为 81,024。

## 4. 验证方法

### 4.1 编译（本次已执行，均通过）

```sh
cd driver
make -C ~/Projects/orangepi-5-max-bsp/third_party/linux \
     M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
modinfo ./anlogic_pci.ko | grep -E 'frame_(width|height|stride|slot_bytes|slots)'

cd app/rs
cargo build --release --target aarch64-unknown-linux-gnu
cargo test --release
```

### 4.2 4K（上板步骤）

```sh
rmmod anlogic_pci 2>/dev/null || true
insmod ./anlogic_pci.ko frame_width=3840 frame_height=2160
dmesg | grep '\[frame\]' | tail -5
# 期望看到 3840x2160、行 stride 4800、R10F 10371072、size 20754432
#（若 dma_alloc_coherent 失败，先查 CMA：当前内核 CONFIG_CMA_SIZE_MBYTES=32）

./frame_grab --info
./link_ctl arm -r A
./frame_grab -n -t 4000 -o 4k.pgm
./link_ctl ack
./link_ctl status
# 0xAC 应为 10371072；0xCC 应为 81024；不一致时 status 会打警告
```

检查 PGM：`head -c 20 4k.pgm` 应看到 `P5\n3840 2160\n255\n`，文件像素部分 8,294,400 B。
若手上有槽内容 dump，再跑：
```sh
./r10f_check <槽内容文件> --width 3840 --height 2160
```

### 4.3 1080p 回归（默认，不能坏）

```sh
rmmod anlogic_pci 2>/dev/null || true
insmod ./anlogic_pci.ko
# 等价于 frame_width=1920 frame_height=1080 frame_stride=0 frame_slots=2
./link_ctl arm -r A
./frame_grab -n -o 1080p.pgm
./link_ctl ack
# 双路
./frame_grab --pair -o a.pgm --out-b b.pgm
```

1080p PGM 像素应为 2,073,600 B；4K 应为 8,294,400 B。两者都不要把
“槽长（R10F 传输字节）”当成“灰度像素数”。

## 5. 启动命令汇总

```sh
# 1080p 双路（默认，已验 demo）
insmod ./anlogic_pci.ko
# 或显式：insmod ./anlogic_pci.ko frame_width=1920 frame_height=1080 frame_stride=0 frame_slots=2

# 4K 单路（两槽，默认 frame_slots=2）
insmod ./anlogic_pci.ko frame_width=3840 frame_height=2160
# 或显式：insmod ./anlogic_pci.ko frame_width=3840 frame_height=2160 frame_stride=4800 frame_slots=2
```

若用 modprobe，把同样的 `options anlogic_pci ...` 写进
`/etc/modprobe.d/anlogic_pci.conf` 后 `modprobe anlogic_pci`；改参数前先 `rmmod`。

## 6. 边界

- 本次只做驱动/用户态的 4K 参数化与编译、单测；**4K 未上板实收**。FPGA 必须实际输出
  R10F：3840×2160、行 stride 4800、payload 10,368,000、槽长 10,371,072，主机才能收到完整帧。
- `frame_stride` 是行 stride（模块参数），`ioctl` 报的 `stride` 是槽步长；
  4K/4KiB 页时分别为 4800 和 10,375,168，不要混用。
- 1080p 默认值、双路行为不变；4K 上板前不要删掉 1080p 的默认路径。
- DDR 双缓冲基址 A0/A1/B0/B1（0x0100_0000 / 0x0200_0000 / 0x0300_0000 / 0x0400_0000）
  属于 `fpga/` RTL 的常量；driver 不引用也不配置它们，本次未改 `fpga/`。
- 0xCC 的期望值由 0xAC 现算；若 FPGA 的 0xAC 尚未按 4K 更新，status 会打
  “MWr != 0xAC/128”的警告，此时不能据此判定数据通路故障。
