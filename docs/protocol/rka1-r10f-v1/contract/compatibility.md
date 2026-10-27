# T03.rk-align-freeze：接口裁决与证据

状态：本包仅固定可恢复起点、合同与合成向量；没有修改生产RTL/驱动，没有执行构建或下载。正式双方G0签收待确认；本包局部G0/G1待三测试，G2—G5按计划不适用。所有源引用相对本run，固定提交见provenance.json。

## 固定版本与责任

FPGA peer=2a4b253d9f2e56a943f1667b2a7a366824dcba34，driver peer=a0215c5a2fcb95bfc867a4768d4831d68edf64a2。本方HEAD=ba0fca5f2c397db88059040b2ffe55c76867da31，origin/main=3862a9beef2081bbd45f8bf54d7f3b4844e3b248：仅记录差异，不同步覆盖。精确快照由git archive对象产生；临时checkout与对象内容可能不同，不作为冻结源码。

本方适配peer的匹配PCIe核/PLL、EP Posted MWr、BAR和doorbell及AXI DDR3-800E后端。本方提供checked CSI、packed RAW10、整帧good/abort裁决、内容CRC与缓冲所有权；RK适配控制位、128B槽偏移、分配容量授权、单槽消费/ACK和严格解析。旧PAXI/HDMI1080p60 002820完整保留；AXI与PAXI不能直接接线替换。peer writer的FRAME_BYTES=W*H和字节提取是8bit帧，不能当RAW10：`references/peer-fpga-2a4b253/rtl/ep_side/ddr/cam_ddr_writer.v:191`；peer DDR参数见`references/peer-fpga-2a4b253/al_ip/ddr3_mc_0/.params_ddr3_mc_0.txt`（800E/AXI=1/PAXI=0）。

## 三层长度与两层128B

packed RAW10布局来自`references/local-inputs/parallel/frame/frame_contract.py:99`附近：P0[9:2],P1[9:2],P2[9:2],P3[9:2],四个低2bit按P0→P3。目标1920×1080，stride=2400B，raw_bytes=payload_bytes=2,592,000B。R10F header=128B，content_bytes=2,592,128B；dma_bytes=ceil(content/4096)*4096=2,592,768B；尾填充640B全0。MWr载荷128B共20,256笔，doorbell另1笔；DDR读4096B共633突发。pusher frame_bytes必须是dma_bytes，不能给raw_bytes或content_bytes：`references/peer-fpga-2a4b253/rtl/ep_side/ddr/ddr_frame_pusher.v:188`。

host槽头仍是四u32=16B（seq/len/flags/reserved），见`references/peer-driver-a0215c5/include/anlogic_pci_ioctl.h:84`。但frameinfo.hdr_bytes改为128B，表示传输数据起点：host [0,16)槽头，[16,128)112B零padding，[128,256)完整R10F头，[256,2592256)有效packed payload，[2592256,2592896)640B DMA尾零。host stride=PAGE_ALIGN(128+2,592,768)=2,596,864B，mailbox页起于stride，单槽总分配2,600,960B（4KiB PAGE_SIZE假设）；槽末其余3968B不是DMA数据。16B host槽头、128B host偏移与128B R10F头不可合并。旧驱动按sizeof(slot_hdr)设置hdr_bytes：`references/peer-driver-a0215c5/src/anlogic_pci_frame.c:161`，地址按hdr_bytes加偏移：`references/peer-driver-a0215c5/src/anlogic_pci_link.c:271`，Rust正确使用返回hdr_bytes的入口：`references/peer-driver-a0215c5/app/rs/src/bin/frame_grab.rs:395`。

CRC32 IEEE reflected poly0xEDB88320，init/final XOR全1。raw_crc32和payload_crc32都只覆盖packed有效载荷；header_crc32将0x58..0x5B清0后覆盖完整128B头；640B transport尾零及host前缀/槽末padding均不入内容CRC。测试向量分别破坏payload/头/尾以验证分层。旧参考build_frame默认pad_to32会将payload padding计入payload_bytes；本包用pack_header明确不含transport尾，1080p尺寸本身整除32。现有parse_frame是草案，不能直接当严格RK发布解析器：必须另检查codec=0、拒bad/partial/oversize、精确长度和尾零。BPR1保留独立，不分配压缩codec，不把BPR1放入codec0。

## CTRL、DMA与所有权

CTRL bit0 enable；bit1测试trigger自清；bit2 reinit toggle；bit3测试pattern；bit4保留读0；bit5 DDR；bit6 push；bit7源；bit8 A/B。实际赋值证据`references/peer-fpga-2a4b253/rtl/ep_side/img/ep_bar_win.v:253`与`references/peer-fpga-2a4b253/rtl/ep_side/img/ep_bar_win.v:355`。RK旧ARM_BITS包含bit4=0x71，见`references/peer-driver-a0215c5/include/anlogic_pci_ioctl.h:216`；回读0x61导致旧条件拒绝。新必需mask=0x61，B另带0x100，触发/reinit不放入稳定arm mask。0x4C是测试源计数，真实相机帧序号A=0xA0/B=0xE0；保留0x64旧VERSION及0xB0..CC语义，旧VERSION不代表新安全扩展存在。

固定128B MWr要求dst%128=0，每笔(addr&4095)+128<=4096。页对齐base+16的第32笔(index31)从0xF90开始跨4KiB：旧+16是明确负例。base+128则每笔不跨4KiB，但不能代替容量、MPS和分配范围授权检查。门铃单DW 4B对齐且独占mailbox页内，不能与DMA传输区重叠；push_dbell相对mailbox偏移32B，见`references/peer-driver-a0215c5/include/anlogic_pci_ioctl.h:130`。地址64bit整帧锁存、容量>=dma_bytes、dst+len无64bit溢出、source base4KiB对齐和地址范围不超过实际DDR容量都是发包前条件。peer只检查dst非0并已有目标/门铃锁存，见`references/peer-fpga-2a4b253/rtl/ep_side/ddr/ddr_frame_pusher.v:669`与`references/peer-fpga-2a4b253/rtl/ep_side/ddr/ddr_frame_pusher.v:681`；不把已有锁存误说成缺失，但新增授权参数与源会话必须同时锁存。

安全扩展拟定于registers.json的0x100..0x11C，原peer这段默认DEAD_BEEF，见`references/peer-fpga-2a4b253/rtl/ep_side/img/ep_bar_win.v:407`；尚未实施。reset容量/会话/授权均0；CTRL 0x61单独不授权发包。单槽one-shot：ARM成功给一帧额度，完成后held，必须匹配(session,seq) ACK才release，并需重新ARM。无完整帧、容量不足、已held或会话不符禁止发包。CANCEL不截断已开TLP，完成开包/排空旧源响应后回idle，失败不门铃。源缓冲writing→complete→held_for_tx→released；双缓冲轮换本身不保证所有权。FPGA先payload后doorbell、同Requester无RO；driver READ_ONCE门铃→dma_rmb→内容验证→发布host槽头/seq；用户态复制/CRC通过后ACK。两次seq相等不能证明传输期间未覆盖。

## 帧来源与验证边界

1920×1080使用真实4K30有效图像的偶数ROI，默认有效区x960/y540；physical dummy/embedded另计。peer SKIP_ROWS=37是peer来源，见`references/peer-fpga-2a4b253/rtl/ep_side/ddr/cam_ddr_writer.v:118`，本板尚未验证，不能抄为板证。phase3来自旧预览配置且需正式帧实查，不能从960×540预览扩大生成1080p。当前两个dma文件均明确synthetic diagnostic。CSI不可反压，CRC/ECC/WC/行数/超时/overflow拒整帧，capture_seq允许缺口；O_frame_good在FE裁决前不发布。完整几何、曝光同步、双路吞吐、G2/G3/G4与RK实收都不由本包证明。

已冻结：版本/职责/非压缩格式/长度/位定义/安全扩展拟定语义。待确认：双方签收、实际相机几何相位、RK通道/内核/MPS/IOMMU/PAGE_SIZE、JTAG器件与硬件窗口。不适用：本包构建、板测、压缩RTL与深度/NPU。
