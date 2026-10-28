# A1U1 历史样例解码包（2026-10-26）

本包只用于离线复现一份 A1U1 文件样例，不连接 FPGA、不访问设备、不做 PCIe 收帧。
解码器只依赖 Python 3 标准库。

## 材料

| 文件 | 说明 |
|---|---|
| `samples/b18_b_1080p.a1u1` | 输入样例，2,594,750 B |
| `decode_sample.py` | 文件输入解码入口 |
| `reference/a1_ref.py`、`reference/a1_inflate_strict.py` | 两份独立解码参考 |
| `expected.json` | 几何、CRC、SHA、来源信息 |
| `provenance.json` | 样例抽取来源与偏移 |
| `SHA256SUMS` | 包内文件校验和 |

## 运行

在本包目录执行：

```sh
python3 -B decode_sample.py --input samples/b18_b_1080p.a1u1 --out-dir out
```

输出：

- `out/frame.raw10`：2,592,000 B，SHA-256
  `4f97a0d5ab9973b45d3e20fd56667deea50df857f2dd14496b5218abfc1ddfd7`；
- `out/result.json`：解码统计与校验结果。

输入 SHA-256：
`2892b3fb66654aa68e4a10609e97bc8737c3b5fb86a9fc4d5cdffc4ae5349d05`。

脚本会检查四项 CRC、严格 DEFLATE 和逆变换。参数：1920×1080、stride 2400、
prep=1、pred=1（flags=3），44 B 头。

## 边界

- 样例是历史 UART 抓取，不是冻结的 PCIe 承载；
- A1U1 头没有相机字段；`source_camera=B` 来自 b18 候选与 sysstat bit12，
  不是头部自带信息；
- 540 个 stored 回退块，无压缩收益；
- `sensor_pixel_oracle_verified=false`，物理 Bayer 相位未知；
- 不证明 30 fps、实际 PCIe 收帧、双路同步或 CSI 像素正确性。
