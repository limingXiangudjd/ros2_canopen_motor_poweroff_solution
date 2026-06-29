# 手动断电脚本 (Bash)
# 用途: 程序异常退出后，手动执行此脚本强制断电
#
# 用法:
#   ./emergency_poweroff.sh [can_interface] [node_id]
#   默认: can_interface=can0, node_id=1

#!/bin/bash

CAN_IF=${1:-can0}
NODE_ID=${2:-1}

# 计算 RPDO1 COB-ID (hex)
RPDO_COB_ID=$(printf "0x%X" $((0x200 + NODE_ID)))

# 计算 NMT Stop 帧 data (hex)
NMT_CMD=$(printf "%02X" 0x02)
NMT_NODE=$(printf "%02X" $NODE_ID)

echo "=== Emergency Motor Power-Off ==="
echo "CAN Interface: $CAN_IF"
echo "Node ID: $NODE_ID"
echo "RPDO1 COB-ID: $RPDO_COB_ID"
echo ""

# Step 1: Halt + Enable (CW=0x010F, Vel=0, Mode=3)
echo "[1/4] Halt + Enable Operation..."
cansend $CAN_IF "${RPDO_COB_ID}#0F01000000000003"
sleep 0.05

# Step 2: Quick Stop (CW=0x0002)
echo "[2/4] Quick Stop..."
cansend $CAN_IF "${RPDO_COB_ID}#0200000000000003"
sleep 0.05

# Step 3: Disable Voltage (CW=0x0000)
echo "[3/4] Disable Voltage..."
cansend $CAN_IF "${RPDO_COB_ID}#0000000000000003"
sleep 0.05

# Step 4: NMT Stop
echo "[4/4] NMT Stop..."
cansend $CAN_IF "000#${NMT_CMD}${NMT_NODE}"

echo ""
echo "=== Power-Off Complete ==="
