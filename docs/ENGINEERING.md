# driver 工程规范

本仓只放 RK3588 主机侧的东西。目标是一眼看得出什么该进、什么不该进。

## 1. 职责边界

- 内核模块：`src/`、`include/`、`Makefile`，只写 C。
- 产品用户态：`app/rs/`，只写 Rust。
- 合同校验：`docs/protocol/*/tools/`，只留能复跑的脚本，Python 标准库优先。
- 合同与交付：`docs/protocol/`、`handoff/`。
- 一次性实验：`experiments/`，本地用，不提交。

算法研究、板测编排、dump、截图、点云都不进本仓。

## 2. 脚本准入

一个脚本只有同时满足下面三条，才允许留在 `docs/protocol/` 或 `handoff/`：

1. README 或 CI 明确引用；
2. 干净机器能按文档复跑；
3. 输入、输出、退出码明确。

否则放 `experiments/`。实验结束：结论写进 `docs/`，脚本随实验一起删除。

## 3. 语言与依赖

- 内核：C，遵守内核风格，`checkpatch` 无新增告警。
- 产品用户态：Rust，交叉编译到 aarch64，静态链接。
- Python：只做离线校验，放在 `docs/protocol/`、`handoff/`；
  标准库优先，numpy/cv2 只允许主机侧，不要求板上安装。

## 4. 不提交的东西

```
*.ko *.o *.mod *.cmd Module.symvers modules.order
app/rs/target/
__pycache__/ *.pyc
*.ply *.npz *.log
experiments/ 下的一切
```

单文件超过 1 MiB 的二进制必须说明来源和用途；
`docs/protocol/` 下的合同向量是例外，必须保留。

## 5. 板测安全

- CTRL(0x44) 只允许通过 `link_ctl arm/ack` 的安全封装写；bit2 电平不许变。
- 任何等待都要有 deadline，不忙等；长采样必须可 Ctrl-C。
- 退出码：0 成功，1 部分失败，2 抓帧失败，3 状态异常。
- 不把旧缓冲、截断帧、CRC 错帧落盘冒充成功。

## 6. 合并门槛

```sh
make -C <内核树> M=$PWD ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
cd app/rs && cargo build --release
python3 docs/protocol/rka1-r10f-v1/tools/r10f_strict.py selftest
python3 docs/protocol/rka1-r10f-v1/contract/check_contract.py
git diff --check
```

## 7. 实验收尾

每次板测结束：

1. 结论写进 `docs/`；
2. 数据只留能复现结论的最小集合；
3. 其余脚本、dump、截图移出仓库或删除；
4. `git status` 除本次有意修改外必须干净。

历史算法参考已整体移出本仓，归档在本机
`../log/archive/driver-app-reference-20261028/`，不入库。
