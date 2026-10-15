#!/bin/bash
# OPi5 Max (RK3588) 侧 PCIe EP 自检脚本 —— 判断米联客 MLK-F9P 的 PCIe EP 卡是否正常
# 用法:  sudo bash opi_pcie_check.sh
set -u
VID="1edb"           # Anlogic PCI-SIG vendor ID
BDF="${1:-0000:01:00.0}"

hr(){ printf '\n========== %s ==========\n' "$1"; }

hr "1. 设备是否在总线上 (lspci)"
if ! lspci -nn | grep -qi "$VID:"; then
  echo "!! 没找到 1edb:* 设备 —— EP 未枚举"
  echo "   排查: FPGA 是否已配置/固化? 12V 是否接? 转接卡/插紧? 试: echo 1 | sudo tee /sys/bus/pci/rescan"
  echo "   dmesg 关键行:"; dmesg | grep -iE "pcie.*(link|fail|error)" | tail -10
  exit 1
fi
lspci -nn | grep -i "$VID:"

hr "2. 链路速率/带宽 + BAR (lspci -vvv)"
sudo lspci -vvv -s "$BDF" | grep -E "LnkCap:|LnkSta:|Region [0-9]:|Capabilities: \[.*\] Power" 
echo "  期望: LnkSta Speed 5GT/s, Width x2 ; Region0/2/4=1MB mem, Region1=256B I/O"

hr "3. 内核枚举日志"
sudo dmesg | grep -iE "1edb|pcie.*link" | tail -10

hr "4. 设备节点 / 已加载驱动"
ls /dev/ | grep -i anlogic || echo "  (还没有 /dev/anlogic*，需先 insmod 驱动)"
lsmod | grep -i anlogic || echo "  (驱动未加载)"

hr "完成。若 1-3 都符合预期 = 硬件/建链正常；模型驱动与 BAR 读写见 driver/ 目录"
