# r10f_view.py —— 把 R10F 帧变成 PNG（纯标准库）

板上直接看帧用，不依赖 numpy / PIL / cargo。

## 两种用法

```bash
# 1) 转换已有 dump（rka1_smoke --out 出来的 2592768 字节，或槽内从 hdr_bytes 起的整段）
python3 tools/r10f_view.py convert /tmp/a.dma -o /tmp/a.png --scale 2

# 2) 直接从设备抓一帧（板子上 root）
sudo python3 tools/r10f_view.py snapshot -o /tmp/snap.png --scale 2 --timeout 8000
```

`--scale 2` 做 2x2 平均（1080p → 540p），`1` 是全分辨率（1920x1080）。

## 注意：帧节点是 one-shot 的

驱动/T07 的推帧是「一个 ARM 额度推一帧」。如果当前没有额度，`snapshot` 会一直等（默认 8s 超时）。
两个办法：

1. 先补一次授权再抓：

   ```bash
   sudo python3 tools/r10f_view.py snapshot -o /tmp/snap.png --timeout 8000 &
   sleep 1
   sudo /root/tools/rka1_smoke --arm --timeout 3000 --no-ack
   wait
   ```

2. 或者先用 `rka1_smoke --arm` 授权、由它自己把帧落盘，再用 `convert` 转 PNG。

## 实现要点（踩过的坑）

- `ANLOGIC_IOCFRAMEINFO` 用的是 `_IOR`（方向位 = 2），不是 `_IOWR`；写成 3 内核会回 `ENOTTY`。
- `struct anlogic_ioc_frame` 的 C `sizeof` 是 **56**（52 + 8 字节对齐补齐），
  ctypes 少一个 pad 字段命令号就对不上。
- 头里的 stride 应当是 `width*5/4`；工具会先做这个自洽检查再解码，避免把坏帧解成花屏。
