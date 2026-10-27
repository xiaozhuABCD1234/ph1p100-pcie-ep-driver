# 历史 RK / PC 离线参考

本目录保存原项目 `parallel/rk` 及其小依赖和合成帧 fixture，保持 `parallel/` 相对结构。它是历史参考，不是生产 DMA 接收器，也不代表 RK 实机性能、真实标定、深度或 NPU 已通过。

当前队友入口是仓库根的 [A1U1 交接包](../../../handoff/rk-a1u1-b18-20261026/README.md)。保留队友当前已经成功的软件和旧 RAW/R10F 入口，再独立增加 A1U1 文件输入支持。

`parallel/rk` 的原 README 和源码按原字节保留，其中路径示例以本目录为参考根。`frame_io.py` 依赖同级 `parallel/frame/frame_contract.py` 和 `parallel/algorithms/image_algorithms_ref.py`，若干 runner 还依赖 `parallel/meta.py`、NumPy/OpenCV。`run_c_cross_check.py` 的 GCC 路径硬编码为旧 Windows 环境，不能直接视为 RK 命令。

旧解析器仅用于对应历史 fixture；正式 R10F 输入先使用仓库根 `tools/pcie_integration/r10f_strict.py` 和 `docs/protocol/rka1-r10f-v1/` 合同。此次不运行深度/NPU，不安装或升级依赖。
