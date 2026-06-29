# CiA402 断电序列技术说明

## CiA402 状态机与断电

CiA402 (IEC 61800-7-2) 定义了伺服驱动的状态机。断电过程涉及以下状态转换：

```
 OPERATION_ENABLED
       │
       │ CW=0x010F (Halt+Enable, Vel=0)     ← Step 1: 减速到 0
       ↓
 OPERATION_ENABLED (halted, velocity=0)
       │
       │ CW=0x0002 (Quick Stop)              ← Step 2: Quick Stop
       ↓
 QUICK_STOP_ACTIVE
       │
       │ (quick_stop_option_code 控制减速方式)
       ↓
 SWITCH_ON_DISABLED
       │
       │ CW=0x0000 (Disable Voltage)          ← Step 3: 关闭使能
       ↓
 SWITCH_ON_DISABLED (power off)
       │
       │ NMT Stop (0x02)                      ← Step 4: 停止通信
       ↓
 STOPPED (所有 PDO 通信停止)
```

## 控制字 (Controlword) 位定义

| Bit | 名称 | 含义 |
|-----|------|------|
| 0 | SO | Switch On |
| 1 | EV | Enable Voltage |
| 2 | QS | Quick Stop (低有效) |
| 3 | EO | Enable Operation |
| 4 | Op-mode specific | New Set Point (位置模式) |
| 5 | Op-mode specific | Change Immediately (位置模式) |
| 6 | Op-mode specific | Abs/Rel (位置模式) |
| 7 | Fault Reset |
| 8 | Halt (低有效) |
| 9 | Op-mode specific |
| 10-15 | Reserved |

### 断电用到的控制字值

| 控制字 | 二进制 | 含义 |
|--------|--------|------|
| 0x010F | 0001 0000 1111 | Enable Operation + Halt → 减速停止 |
| 0x0002 | 0000 0000 0010 | Quick Stop=0 (触发快速停止) |
| 0x0000 | 0000 0000 0000 | Disable Voltage=0 + Quick Stop=0 → 关闭功率 |

## RPDO1 帧格式

RPDO1 (Receive Process Data Object 1) 是主站发送给从站的实时控制帧：

```
COB-ID: 0x200 + node_id (如 node_id=1 → 0x201)

Data (7 bytes):
  Byte 0-1: Controlword (uint16, little-endian)
  Byte 2-5: Target Velocity (int32, little-endian)
  Byte 6:   Mode of Operation (int8)
```

### 为什么 RPDO1 包含 3 个对象？

CiA402 设备通常将 Controlword、Target Velocity、Mode of Operation 映射到同一个 RPDO，
这样一帧 CAN 报文就能同时设置控制字、目标速度和运行模式，保证原子性。

## NMT 命令

NMT (Network Management) 是 CANopen 的网络管理协议：

| COB-ID | Data | 命令 | 效果 |
|--------|------|------|------|
| 0x000 | 0x01, node_id | Start Remote Node | 进入 Operational |
| 0x000 | 0x02, node_id | Stop Remote Node | 进入 Stopped |
| 0x000 | 0x80, node_id | Enter Pre-Operational | 进入 Pre-Operational |
| 0x000 | 0x81, node_id | Reset Node | 硬件复位 |
| 0x000 | 0x82, node_id | Reset Communication | 通信复位 |

断电序列使用 NMT Stop (0x02) 使设备进入 Stopped 状态，停止所有 PDO 通信。

## SocketCAN 编程要点

### CAN socket 创建流程

```c
// 1. 创建 socket
int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

// 2. 指定接口
struct ifreq ifr;
strcpy(ifr.ifr_name, "can0");
ioctl(sock, SIOCGIFINDEX, &ifr);

// 3. 绑定
struct sockaddr_can addr;
addr.can_family = AF_CAN;
addr.can_ifindex = ifr.ifr_ifindex;
bind(sock, (struct sockaddr*)&addr, sizeof(addr));

// 4. 发送
struct can_frame frame;
frame.can_id = 0x201;
frame.can_dlc = 7;
memcpy(frame.data, data, 7);
write(sock, &frame, sizeof(frame));
```

### 关键特性

- **内核级**：SocketCAN 在 Linux 内核中实现，不依赖用户态库
- **无阻塞**：`write()` 通常 < 1ms（内核缓冲区）
- **不依赖 ROS**：与 ROS executor/service 完全无关
- **需要 CAP_NET_RAW**：需要 root 或 `cap_net_raw` capability

### CAP_NET_RAW 设置

```bash
# 方式1: setcap (推荐)
sudo setcap cap_net_raw+ep /path/to/executable

# 方式2: setuid (不推荐，安全风险)
sudo chmod u+s /path/to/executable

# 方式3: canopen-launch-wrapper (带 ambient capability)
sudo setcap cap_net_raw+eip /usr/local/bin/canopen-launch-wrapper
```
