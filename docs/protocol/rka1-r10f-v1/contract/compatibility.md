# RKA1 / R10F v1 兼容性与实施状态

本文件只描述当前驱动实际使用并由板测确认过的布局与语义。线协议字段仍以
`registers.json` 和 `acceptance-vectors.json` 为准。

## R10F v1 帧

- 头 128 B，魔术字 `R10F`，版本 1.0，非压缩，codec 0；
- 1920×1080、packed RAW10、valid_bits=10、packing=0；
- stride 2400 B/行，payload = raw = 2,592,000 B；
- 传输长度 2,592,768 B = 128 B 头 + payload + 640 B 尾零（到 4 KiB 对齐）；
- 头的 0x58 是 header CRC，0x50/0x54 是 raw/payload CRC；两者都覆盖 packed payload；
- 头 CRC 计算前先把 0x58..0x5B 清 0；尾零和 host 槽前缀不参与内容 CRC。

实际交付的 1080p ROI 帧，CFA 为 **BGGR**（(0,0)=B，G 在 (0,1)/(1,0)）。
当前 R10F 头 0x28 写 3(GBRG)，这是 RTL 的 `SRC_BAYER_PHASE=0` 占位按 ROI 原点
奇偶推导出来的值，不是测量值；去马赛克不能直接信这个字段。

以上布局由本板 2026-10-28 抓取的真实帧经 `frame_grab` 与 `r10f_check` 逐字节复算通过。

## Host 槽布局

驱动用 `dma_alloc_coherent` 分配一块一致性内存：

- 槽头 16 B：seq/len/flags/reserved；
- 槽内数据偏移 128 B；
- [128, 2592256) 为 R10F 头 + payload，之后 640 B 尾零；
- 槽步长 2,596,864 B（4 KiB 对齐）；
- mailbox 在槽区之后独立一页，偏移由 ioctl 报出的 `mb_off` 给出；
- 单槽总分配 2,600,960 B（1080p、4 KiB 页）。

`frame_ioc` 的 `hdr_bytes` 是 128，不是 16；16 B 槽头与 128 B R10F 头是两层，不能合并。

## CTRL 位

当前实现使用：

- bit0 enable；
- bit1 trigger，测试图单帧触发，自清；
- bit2 重触发电平，写它会导致 XCLR 边沿，任何 CTRL 写入都必须保持 bit2 不变；
- bit5 DDR 搬运；
- bit6 push；
- bit8 选 A/B（0=A，1=B）。

A 路推帧取值 `0x61`，B 路 `0x161`，空闲 `0x21`。bit4 在当前位流保留、读回 0。

## RKA1 one-shot

- 每次 `ARM` 只给一帧额度；
- 一帧推完进入 HELD，必须用门铃里的 `push_done` 与 session 匹配 `ACK` 才释放；
- ACK 后才能再次 ARM；
- `CANCEL` 完成已开包、排空旧响应后回 idle，不发成功门铃；
- 驱动 probe 会初始化推帧并 ARM 一次，因此上电后通常已有一帧可读；
- 用户态入口是 `link_ctl arm` / `link_ctl ack`，写 CTRL 前后做 bit1/bit2 门禁和回读复核。

常见 `0xC8`：

- `5`：有完整帧但没有 ARM 额度；
- `8`：HELD 期间源又出帧，已忽略。

## 已验证与未验证

已在 RK3588 板验证：

- PCIe Gen2 ×2、BAR0 1 MiB、MAGIC `0x50434946`；
- 模块加载/卸载、三个设备节点；
- A 路 1920×1080 R10F 实收，2,592,768 B，三层 CRC 通过；
- `frame_grab` 输出 PGM 2,073,600 B 像素；
- `link_ctl arm -r A` → 推帧 → `link_ctl ack` 完整通过。

未在本轮验证：

- B 路实收（代码支持 `arm -r B`）；
- 4K 几何；
- MSI 中断路径；
- 100 帧 / 10 min 长稳；
- 相机重新固定后未再做白参考复测；
- 真实标定、双路同步与吞吐。
