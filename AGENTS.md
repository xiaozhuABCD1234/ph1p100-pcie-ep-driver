# 仓库协作约定

- FPGA 权威原仓：https://github.com/xiaozhuABCD1234/ph1p100-pcie-ep-fpga
- RK 权威原仓：https://github.com/xiaozhuABCD1234/ph1p100-pcie-ep-driver
- 本次用户授权直接更新上述原仓 main，不新建 fork。
- 后续修改以当前仓库和用户当次授权为准，不把旧副本当成最新成功工程。
- 保留队友已经成功的接收、RAW/R10F 和解码入口；新格式增加独立入口。
- 队友当前入口：handoff/rk-a1u1-b18-20261026/README.md。
- 同步范围和已知局限：docs/PROJECT_SYNC_20261027.md。
- 复制来源和字节 SHA：docs/sync-20261027-files.json。
- PC 离线通过不能替代 RK 编译、加载、DMA、吞吐或恢复实测。
- 不自动升级依赖、加载驱动、操作硬件或覆盖队友成功配置。
- 禁止批量删除文件或目录，也禁止将批量删除包装为循环、脚本或子进程。
- 需要删除时，一次只能删除一个明确路径的文件；批量删除请用户手动处理。
