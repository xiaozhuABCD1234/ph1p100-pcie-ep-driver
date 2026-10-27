# RK 原仓材料同步记录（2026-10-27）

本次将 `D:/fpga/ph1p100` 中已经存在的更新同步到 RK 原仓 `main`。用户授权直接更新两个原仓，不创建 fork。FPGA 原仓为 [xiaozhuABCD1234/ph1p100-pcie-ep-fpga](https://github.com/xiaozhuABCD1234/ph1p100-pcie-ep-fpga)，RK 原仓为 [xiaozhuABCD1234/ph1p100-pcie-ep-driver](https://github.com/xiaozhuABCD1234/ph1p100-pcie-ep-driver)。此次 RK 同步基线为 `a0215c5a2fcb95bfc867a4768d4831d68edf64a2`。

队友从 [handoff/rk-a1u1-b18-20261026/README.md](../handoff/rk-a1u1-b18-20261026/README.md) 开始，保留自己已经成功的收图、RAW/R10F 和解码软件，再增加 A1U1 文件入口。用户此前已确认旧真实收图和解码已通；此次没有在队友 RK 上复验，也不以本仓历史诊断工具替换队友成功工程。环境和成功命令记录见 [RK基线复现_README.md](RK基线复现_README.md)。

## 同步来源

| 本仓路径 | 原项目来源与性质 |
| --- | --- |
| `Makefile`、`include/`、`src/`、`app/rs/` 中本次 11 个代码文件 | `workflow/runs/20261017-011744-rk-fpga-align/raw/rk-driver-src/` 中 T07 主补丁加 T11 R10F 增量的最终历史候选，8 改 3 增 |
| `docs/protocol/rka1-r10f-v1/` | FPGA 原仓工作区同名目录，完整合同、合成 DMA 向量、SHA 镜像；协议主源仍在 FPGA 原仓 |
| `tools/pcie_integration/r10f_strict.py`、`r10f_tools_consistency.py` | 原项目独立严格解析与 Python/Rust 字段静态交叉检查 |
| `handoff/rk-a1u1-b18-20261026/` | 原项目最新同名交接包，排除所有 `out*`、`__pycache__` 和 `.pyc` |
| `app/reference/stereo/parallel/` | 历史 `parallel/rk`、三个小依赖与 `frame_contract_20261015` 合成 fixture，保持相对结构，仅供参考 |
| `docs/RK基线复现_README.md` | 原项目同名文档，保留当时事实与适用边界 |

逐文件来源相对路径、分组、字节数、源/目标 SHA-256、换行统计见 [sync-20261027-files.json](sync-20261027-files.json)。复制保持原字节，没有源码语义修复。清单中的 SHA 描述本次复制工作树字节；现有驱动文本沿用原 Git 行尾行为。根 `.gitattributes` 只为新协议目录和 A1U1 交接包禁止文本换行转换；包内 `.gitattributes` 同时保留。

## 旧 R10F 与新 A1U1

RKA1/R10F v1.0 保持 RAW、codec0、packed RAW10 的历史合同和严格校验器，不能拿 A1U1 替换它的帧格式或旧成功入口。主机槽 128B 数据起点与 DMA 正文内部 128B R10F 头属于两层。

A1U1 包是历史 b18 相机 B 的真实 UART 文件样例，1920×1080、stride=2400、RAW=2,592,000B。四项 CRC 和逆变换用于文件复现；540 个 stored 回退块没有压缩收益。Bayer 物理相位未知，sensor pixel oracle 未验证；A/B 身份来自候选及状态记录，A1U1 头本身没有相机字段。它尚非冻结的 PCIe 压缩承载，不证明 30fps、真实 PCIe 或双路同步。

## 现有代码的未完成项

1. `app/rs/src/bin/frame_grab.rs` 仍是单帧诊断工具，仅在 magic 恰为 `R10F` 时调用严格解析，损坏 magic 仍可能落入旧路径；完整的 ARM → 等待 held → 复制/严格校验 → 匹配 session/seq → ACK → 再 ARM 循环尚未接入。
2. 同一工具的 PGM 输出不是正确的 packed RAW10 预览。正式输入应保存原始 DMA/R10F 与元数据，解包后另生成预览，不能将原字节当作可信灰度像素。
3. `src/anlogic_pci_link.c` 的 disarm 在 CANCEL 未确认时仍继续关模式；`src/anlogic_pci_hw.c` 随后可能释放 DMA 缓冲。排空失败、退出/拔链/remove/reset 的安全生命周期尚需专门修复和 RK/FPGA 联调。

本次只同步已有候选，不声称这些缺口已经修复或 RK 内核/Rust 编译、加载、DMA 实收、吞吐、100帧/10min、长期稳定与恢复已通过。`app/reference/stereo` 也不启动深度/NPU 任务。

## 从仓库根执行离线复核

以下输出目录为本地 ignored 目录，不能替代队友 RK 的运行记录；不要覆盖已有输出。

```sh
python3 -B docs/protocol/rka1-r10f-v1/contract/check_contract.py
python3 -B tools/pcie_integration/r10f_strict.py selftest --run-dir docs/protocol/rka1-r10f-v1
python3 -B tools/pcie_integration/r10f_tools_consistency.py --rust-src app/rs/src/rka1.rs
python3 -B handoff/rk-a1u1-b18-20261026/decode_sample.py --input handoff/rk-a1u1-b18-20261026/samples/b18_b_1080p.a1u1 --out-dir out-sync-20261027/a1u1
```

旧工具默认仍指向原项目的历史 run；在本仓运行必须显式给出上面的 `--run-dir` 和 `--rust-src`。合同脚本只核对文档/fixture，不执行生产 RTL、内核驱动或硬件。队友在实际 RK 上重新记录依赖、命令、输入/输出 SHA 和耗时，PCIe 实收与恢复留待具备真实硬件时联调。

## 本次实际验证

2026-10-27，在 Windows 本机 Python 3.11 执行上述工具；所有命令返回码为 0。本机耗时不代表 RK 速度。

| 检查 | 本次结果 |
| --- | --- |
| 复制源/目标 SHA | 52/52 文件一致，零换行变更 |
| 协议 SHA256SUMS | 9/9 通过 |
| A1U1 SHA256SUMS | 9/9 通过 |
| 合同与 fixture 脚本 | 84 项，0 失败；只核对合同/fixture |
| R10F 严格解析 selftest | 38 项，0 失败 |
| Python/Rust 字段偏移 | 25 个 Rust 字段无偏移差异；4 个 Python 元数据字段尚未由 Rust 读取，此检查不是编译 |
| A1U1 真样例 | 四 CRC、长度与 RAW SHA 符合 expected.json，RAW 2,592,000B；文件读取/解码 0.592788s（不含输出文件写入） |
| A1U1 期望值/负例复核 | 17 项，0 失败；拒绝截断、头 CRC 错误、码流 CRC 错误、非法码流长度 |

实际调用使用 `python -X utf8 -B`；合同脚本的 `--out` 和两工具的 `--json-out` 写入 ignored 目录 `out-sync-20261027/`，A1U1 输出为其 `a1u1/` 子目录。该目录另存复制/SHA、期望值/负例和汇总 JSON，不作为生产源码提交。

本机没有 `cargo`，未安装工具链、未编译 Rust，也没有 RK 内核构建树与硬件；RK 编译/加载/文件速度、PCIe DMA、持续吞吐、退出恢复、真实标定/深度/NPU 全部未测。历史候选的接收循环与 CANCEL 生命周期缺口保持原状。
