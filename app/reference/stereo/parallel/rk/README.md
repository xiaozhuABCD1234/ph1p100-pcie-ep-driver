# parallel/rk — RK 侧离线可运行起点

给 RK 执行者（队友）与离线验证用的最小可跑流程。**全部离线、不需要开发板**。

## 为什么可信：灰度路径与 FPGA 金参考逐像素一致

`frame_io.py` 用 numpy 重新实现了帧解码 / 3×3 双线性去马赛克 / 固定亮度映射，
并在自检里断言它与 `parallel/algorithms` 的纯 Python 金参考**逐像素相同**
（四个 Bayer 相位全部比对）。因此：

> 同一份帧合同向量，FPGA 侧与 RK 侧必须得到**同一组 Y8**。若两侧对不上，
> 一定是实现不同，而不是"算法本来就该不同"。

## 三个文件

| 文件 | 作用 |
|---|---|
| `frame_io.py` | 帧合同 `.bin` 读取 → packed RAW10 解包 → 去马赛克 → Y8；自检与金参考对比 |
| `run_rk_offline.py` | 合成双目（阶梯视差真值）→ 灰度 → `StereoBM`/`StereoSGBM` → 视差误差；输出 PNG + JSON |
| `stereo_calib_template.py` | 真实标定板图片的双目标定入口（需要你/队友提供实拍图） |

```powershell
python parallel/rk/frame_io.py
python parallel/rk/run_rk_offline.py
python parallel/rk/stereo_calib_template.py --left "L*.png" --right "R*.png" --cols 9 --rows 6 --square-mm 25
```

## 当前实测结果（本机，合成数据）

`run_rk_offline.py`：320×240、`numDisparities=64`、左半真值视差 16 / 右半 48
（只统计避开中缝与边界的内部区域）：

| 方法 | 有效比例 | bad2 比例（\|误差\|>2） | RMSE |
|---|---:|---:|---:|
| StereoBM | 0.6975 | 0.0007 | 0.849 |
| StereoSGBM | 0.7812 | 0.0000 | 0.013 |

这些数字只证明**离线流程能跑通且灰度路径一致**，**不是** RK 实机性能，也不是实拍精度。

## 队友接手时的最短路径

1. 把 `run_rk_offline.py` 里的 `synth_scene()` 换成"从 PCIe 收到的左右帧"
   （或先读 `parallel/evidence/frame_contract_20261015/*.bin` 走通解析）。
2. 用实拍标定板跑 `stereo_calib_template.py` 得到 `R1/R2/P1/P2/Q`。
3. 用 `cv2.remap` 校正左右图 → 再跑 BM/SGBM → `reprojectImageTo3D` 得到点云/距离。
4. 记录：标定重投影误差、基线、已知距离目标的偏差、有效视差比例、各段耗时。

## 边界

- 合成数据不能证明实拍精度、曝光同步或动态场景表现。
- 标定模板未在本机用真实图片运行（没有实拍标定图），只验证了参数解析与无图时的错误路径。
- 未与队友冻结接口；`codec_id`/`block_count` 等压缩字段属 F2。
