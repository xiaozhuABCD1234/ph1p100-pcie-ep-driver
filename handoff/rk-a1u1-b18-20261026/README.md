# RK A1U1 历史真样例交接（2026-10-26）

目标：RK 可独立并行适配，配合 10/27 FPGA 功能闭环。旧真实收图/解码已通过，不重做旧驱动。本包仅依赖 Python 3 标准库，无需 FPGA。

材料：`samples/b18_b_1080p.a1u1`、`reference/a1_ref.py`、`reference/a1_inflate_strict.py`、`decode_sample.py`、`expected.json`、`SHA256SUMS`。

在本包目录执行，并记录耗时：

```sh
python3 -B decode_sample.py --input samples/b18_b_1080p.a1u1 --out-dir out
```

脚本检查四项 CRC、严格 DEFLATE 和逆变换，输出 `out/frame.raw10`、`out/result.json`。参数：1920×1080，stride=2400，prep=1、pred=1（flags=3）。

| 文件 | 字节数 | SHA-256 |
| --- | ---: | --- |
| 输入 A1U1 | 2594750 | `2892b3fb66654aa68e4a10609e97bc8737c3b5fb86a9fc4d5cdffc4ae5349d05` |
| 输出 RAW10 | 2592000 | `4f97a0d5ab9973b45d3e20fd56667deea50df857f2dd14496b5218abfc1ddfd7` |

RK 按优先级完成：

1. 跑真样例，核对 SHA、结果和耗时。
2. 给现有 RK 程序增加 A1U1 输入，保留旧 RAW/R10F 入口。
3. 输出 RAW，可选生成预览；物理 Bayer 相位尚未验证，暂记未知。
4. 对截断、CRC 错误、非法长度作基本拒绝。
5. 新建 `results/rk-<date>-<name>/`，将成功版本、完整命令和结果回存主仓库，不放秘密或环境 dump。

A1U1 是当前 UART 导出封装，尚非冻结的 PCIe 承载；后续替换设备输入并补 camera/session/mode 元数据。样例由真抓包提取 44B 头和 zlib；source B 由 b18 候选及 status bit12 确认，头本身没有 A/B 字段。

此样例使用 540 个 stored 回退块，无压缩收益，只用于历史样例复现；不证明 30fps、实际 PCIe 或双路同步，也不证明入口 packed 像素已正确对应 CSI。源像素正确性仍由 FPGA 集成修复。
