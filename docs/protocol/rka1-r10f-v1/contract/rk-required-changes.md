# RK必改清单（目标driver a0215c5，尚未应用）

本清单是下一包的明确补丁输入；当前没有写入队友仓库、应用补丁、RK编译/安装或发送消息。新FPGA必须同时实现registers.json安全扩展后才允许formal模式；legacy diag不能绕过安全门槛。

| 必改项 | 精确位置与当前行为 | 所需结果/用户态联动 |
|---|---|---|
| CTRL | `references/peer-driver-a0215c5/include/anlogic_pci_ioctl.h:205`、`references/peer-driver-a0215c5/include/anlogic_pci_ioctl.h:216`；`references/peer-driver-a0215c5/src/anlogic_pci_link.c:321` | bit4废弃不要求读1；ARM_BITS=0x61；新增CAM_AB=0x100，B读取0xE0..EC；bit2只作reinit toggle。cam_grab顶部旧bit4说明与常量同改，见`references/peer-driver-a0215c5/app/rs/src/bin/cam_grab.rs:10`。 |
| 槽偏移/容量 | `references/peer-driver-a0215c5/src/anlogic_pci_frame.c:161`、`references/peer-driver-a0215c5/src/anlogic_pci_frame.c:195`、`references/peer-driver-a0215c5/src/anlogic_pci_frame.c:201` | 保留16B struct，hdr_bytes=128，前缀padding112B清0，slot_bytes=2592768，width1920/height1080，slots必须=1；运行时PAGE_SIZE核对，安全算stride/size无溢出，非法参数返回错误而不静默夹紧。4KiB时stride2596864/size2600960。 |
| 地址编程 | `references/peer-driver-a0215c5/src/anlogic_pci_link.c:271`、`references/peer-driver-a0215c5/src/anlogic_pci_link.c:279` | 使用dma_alloc_coherent返回的PCIe DMA地址，不用CPU物理地址替代；dst=base+128；db=base+mb_off+offsetof(push_dbell)=base+2596864+32；低/高地址均写读回。限制任意PUSHCFG到授权分配且禁busy/held改有效配置，见`references/peer-driver-a0215c5/src/anlogic_pci_link.c:601`。保留64bit carry测试。 |
| 默认禁推/安全扩展 | `references/peer-driver-a0215c5/src/anlogic_pci_link.c:316` | 禁自动CTRL模式即持续DMA；先验证EXT_MAGIC=0x31414B52/EXT_VERSION=0x10000，确认BAR映射长度>=0x120（原WIN_END只有0xD0），配置CAP_BYTES/SESSION及地址，校验MPS>=128、single slot，再ARM_ONE_SHOT；旧peer扩展读DEAD_BEEF必须拒绝。 |
| 发布/回收 | `references/peer-driver-a0215c5/src/anlogic_pci_frame.c:126`、`references/peer-driver-a0215c5/src/anlogic_pci_frame.c:135` | 保留READ_ONCE+dma_rmb；完成只表示传输终结，完整R10F检查后才COMPLETE；len语义为本次DMA有效传输范围dma_bytes，解析以R10F payload_bytes截取。held时禁止覆盖。用户态校验/保存完成才匹配ACK_SESSION/ACK_SEQ并ACK_RELEASE，之后再ARM下一帧。无ACK接口时只能一次传输后停住；不能用sleep/两次seq替代消费授权。 |
| 严格解析 | `references/peer-driver-a0215c5/app/rs/src/bin/frame_grab.rs:395`、`references/peer-driver-a0215c5/app/rs/src/bin/frame_grab.rs:383` | 使用返回hdr_bytes；非法len拒绝，取消夹紧后当成功的路径。用R10F magic/version/header128/format0/codec0/valid_bits10/packing0、geometry/stride/raw=payload、flags、session/seq及三CRC验证，要求dma精确长、所有尾填充0；未知codec拒绝。ROI/Bayer只能按验证过的源模式解释。 |
| 生命周期 | `references/peer-driver-a0215c5/src/anlogic_pci_link.c:424`及卸载/removed路径 | close/unload/link reset时CANCEL→等待busy/draining清零→屏蔽bus master/确认设备不再DMA→释放coherent内存；停止不可仅清push mode就马上free。重连用新session，拒绝旧doorbell/ACK/DDR响应。 |

保持现有ioctl编号与结构体布局，包括16B槽头与64B mailbox。新ACK可经现有受锁保护的BAR控制ioctl封装，不把新字段塞进旧结构体引起32/64bit ABI漂移。新字段宏、扩展窗口、safe arm helper及Rust parser/ack wrapper在后续补丁一次交付；此包不声称已有可运行补丁。

RK执行前由操作者提供真实设备BDF、driver/kernel提交、uname/getconf PAGE_SIZE、PCIe MPS及DMA mask/IOMMU状态；这些是需要采集的事实，不预设RK IP或COM号。目标板实收验收：同一冻结bit/driver至少100帧，长度/逐样本/CRC/seq/session一致，坏帧不发布，慢消费者不覆盖，恢复1次；10min短稳独立记录计数和帧率。此处均为后续操作验收，当前not_run。

先执行contract/check_contract.py确认离线向量，再由后续包产生并apply-check精确patch；不要现在将本清单的建议当上机结果。
