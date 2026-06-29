# ROS 2 CANopen 电机安全断电方案

> **一句话摘要**：ROS 2 程序退出时，`ros2_control` 的 lifecycle 回调可能不被执行或执行不完整，导致 CANopen 伺服电机不掉电持续运转。本方案通过 SocketCAN 直接发送 CiA402 断电指令序列（Halt → Quick Stop → Disable Voltage → NMT Stop），确保在任何退出路径下电机都能安全停止。

## 📋 目录

- [问题背景](#问题背景)
- [根因分析](#根因分析)
- [解决方案架构](#解决方案架构)
- [核心实现](#核心实现)
- [使用方法](#使用方法)
- [断电时序详解](#断电时序详解)
- [退出路径覆盖矩阵](#退出路径覆盖矩阵)
- [为什么走 SocketCAN 而非 ROS Service](#为什么走-socketcan-而非-ros-service)
- [效果对比](#效果对比)
- [适用范围与限制](#适用范围与限制)
- [FAQ](#faq)

---

## 问题背景

### 现象

在 ROS 2 Humble 中使用 `ros2_control` + `ros2_canopen` 驱动伺服电机时，`Ctrl+C` 停止程序后：

- ✅ 程序正常退出，无崩溃
- ❌ **电机仍在运转**，保持最后一个速度命令
- ⚠️ 必须手动断电才能停止电机

### 危害

| 场景 | 后果 |
|------|------|
| 速度模式运行中退出 | 电机持续运转，设备失控移动 |
| 位置模式运行中退出 | 电机完成最后位置命令后保持使能，意外触发可能突然运动 |
| 紧急停止按钮失效 | 如 ROS 节点占用 CAN 总线，硬件急停可能被阻塞 |

### 环境

| 组件 | 版本 |
|------|------|
| ROS 2 | Humble |
| ros2_control | 3.x |
| ros2_canopen | 0.2.13 |
| 伺服电机 | 雷赛 LD2-CAN (CiA402 兼容) |
| CAN 接口 | SocketCAN (can0) |

---

## 根因分析

### ROS 2 Lifecycle 回调的局限性

`ros2_control` 的 `SystemInterface` 提供 7 个 lifecycle 回调：

```
on_configure → on_activate → [运行] → on_deactivate → on_cleanup → on_shutdown
                                                           ↑
                                                    on_error (异常路径)
```

**问题**：`Ctrl+C` 触发 SIGINT 后，ROS 2 的 shutdown 流程并非总是完整执行所有回调：

| 退出方式 | on_deactivate | on_cleanup | on_shutdown | 电机断电? |
|---------|:---:|:---:|:---:|:---:|
| `Ctrl+C` (正常 SIGINT) | ✅ | ⚠️ 不确定 | ⚠️ 不确定 | ❌ 可能不掉电 |
| `kill -9` (SIGKILL) | ❌ | ❌ | ❌ | ❌ 不掉电 |
| 节点崩溃 (segfault) | ❌ | ❌ | ❌ | ❌ 不掉电 |
| `ros2 lifecycle set` | ✅ | ✅ | ✅ | ✅ 正常断电 |

### 根本原因

1. **SIGINT 时间紧迫**：ROS 2 收到 SIGINT 后给 lifecycle 节点有限时间完成 shutdown，超时后强制杀死进程
2. **ROS Service 不可用**：shutdown 过程中 ROS 通信基础设施（executor、service server）可能已销毁，`sdo_write()` Service 调用会失败
3. **SDO 通信太慢**：单个 SDO 请求 20-500ms，4 步断电序列需要 1-2 秒，可能来不及完成就被强制杀死
4. **CAN 总线可能被占用**：如果有其他 CAN 节点正在通信，SDO 请求可能排队等待

### 代码级追踪

```
Ctrl+C
  → ROS 2 SIGINT handler
  → rclcpp::shutdown()
  → lifecycle manager 触发 on_deactivate()
    → 需要发送 4 条 CAN 帧断电
    → 但 ROS Service 可能已不可用
    → SDO 超时 → 断电序列中断
  → 进程被杀死
  → 电机保持最后状态 (运转中)
```

---

## 解决方案架构

### 核心思路

**不依赖 ROS 通信基础设施，通过 SocketCAN 直接发送 CAN 帧断电。**

SocketCAN 是 Linux 内核级 CAN 驱动，只要进程还活着、CAN 接口还在线，就能发 CAN 帧。不依赖 ROS executor、service server、publisher 等任何 ROS 机制。

### 三层断电防线

```
第 1 防线: on_deactivate() / on_shutdown()     ← 正常退出路径
  ↓ 若未执行
第 2 防线: on_error()                           ← 异常退出路径
  ↓ 若未执行
第 3 防线: 析构函数 ~IslCHardwareInterface()     ← 最后保障
  ↓ 若仍未执行
(电机保持最后状态 — 需要硬件急停按钮作为最终手段)
```

### 断电序列设计

基于 CiA402 标准的三步渐进断电 + NMT Stop：

```
Step 1: Halt + Enable (CW=0x010F, Vel=0)    → 减速停止
Step 2: Quick Stop (CW=0x0002, Vel=0)        → 快速停止
Step 3: Disable Voltage (CW=0x0000, Vel=0)   → 关闭使能
Step 4: NMT Stop (COB-ID=0x000, data=0x02)   → 停止所有通信
```

**为什么需要 4 步而不是 1 步？**

| 步骤 | 作用 | 跳过的风险 |
|------|------|-----------|
| Halt+Enable | 减速到 0（使用电机内部减速度） | 突然停止可能损坏机械结构 |
| Quick Stop | 触发 CiA402 Quick Stop 功能（更激进的减速） | 电机可能仍在使能状态 |
| Disable Voltage | 关闭电机功率输出 | 电机完全失去使能 |
| NMT Stop | 停止所有 PDO 通信 | 防止残留 PDO 帧重新触发运动 |

---

## 核心实现

### 文件结构

```
motor_poweroff_controller/
├── CMakeLists.txt
├── package.xml
├── include/motor_poweroff_controller/
│   └── motor_poweroff_controller.hpp     # 核心头文件
├── src/
│   ├── motor_poweroff_controller.cpp     # 核心实现
│   └── poweroff_node.cpp                 # 独立断电节点
├── config/
│   └── poweroff_config.yaml              # 配置文件
└── launch/
    └── poweroff.launch.py                # launch 文件
```

### 关键代码：SocketCAN 断电函数

```cpp
/**
 * @brief 通过 SocketCAN 直接发送 CAN 帧
 *
 * 不依赖 ROS 通信基础设施，只要进程活着就能发 CAN 帧。
 * 这是断电方案的核心保障。
 */
bool MotorPoweroffController::send_can_frame(uint32_t can_id,
                                              const uint8_t* data, uint8_t len)
{
  std::lock_guard<std::mutex> lock(can_mutex_);

  // 惰性初始化 CAN socket
  if (can_socket_ < 0) {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) return false;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;
  frame.can_dlc = len;
  std::memcpy(frame.data, data, len);

  return ::write(can_socket_, &frame, sizeof(frame)) == sizeof(frame);
}

/**
 * @brief 发送 RPDO1 帧 (Controlword + TargetVelocity + Mode)
 *
 * RPDO1 COB-ID = 0x200 + node_id
 * 帧格式: [CW_lo, CW_hi, Vel_b0, Vel_b1, Vel_b2, Vel_b3, Mode] = 7 bytes
 * 延迟 < 1ms（直接内核级 CAN 发送，不经 ROS/SDO 栈）
 */
bool MotorPoweroffController::send_rpdo1(uint16_t controlword,
                                          int32_t target_velocity, int8_t mode)
{
  uint32_t can_id = 0x200 + node_id_;
  uint8_t data[7];
  data[0] = controlword & 0xFF;
  data[1] = (controlword >> 8) & 0xFF;
  data[2] = target_velocity & 0xFF;
  data[3] = (target_velocity >> 8) & 0xFF;
  data[4] = (target_velocity >> 16) & 0xFF;
  data[5] = (target_velocity >> 24) & 0xFF;
  data[6] = static_cast<uint8_t>(mode);
  return send_can_frame(can_id, data, 7);
}

/**
 * @brief 三步渐进断电序列
 *
 * Step 1: Halt + Enable (CW=0x010F) — 减速停止
 * Step 2: Quick Stop (CW=0x0002)     — 快速停止
 * Step 3: Disable Voltage (CW=0x0000) — 关闭使能
 * Step 4: NMT Stop                    — 停止所有通信
 *
 * 每步间隔 50ms，总耗时 ~200ms
 * 走 SocketCAN 直接发送，不依赖 ROS Service
 */
void MotorPoweroffController::emergency_poweroff()
{
  // Step 1: Halt + Enable (使用电机的减速度平滑停止)
  send_rpdo1(CMD_ENABLE_OPERATION | 0x0100, 0, MODE_VELOCITY);  // CW=0x010F
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Step 2: Quick Stop (更激进的减速)
  send_rpdo1(CMD_QUICK_STOP, 0, MODE_VELOCITY);  // CW=0x0002
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Step 3: Disable Voltage (关闭功率输出)
  send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);  // CW=0x0000
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Step 4: NMT Stop (停止设备所有 PDO 通信)
  uint8_t nmt_data[2] = {0x02, node_id_};
  send_can_frame(0x000, nmt_data, 2);
}
```

### Lifecycle 回调中的断电调用

```cpp
hardware_interface::CallbackReturn on_deactivate(
  const rclcpp_lifecycle::State &) override
{
  // 1. 停止 detached 线程（防止 use-after-free）
  shutdown_detached_threads();

  // 2. 标记服务不可用（让异步读取线程退出）
  services_available_.store(false);

  // 3. 停止异步读取线程
  stop_async_read_thread();

  // 4. ★ 通过 SocketCAN 直接断电（不依赖 ROS Service）
  emergency_poweroff();

  // 5. 关闭 CAN socket
  {
    std::lock_guard<std::mutex> lock(can_mutex_);
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn on_shutdown(
  const rclcpp_lifecycle::State &) override
{
  shutdown_detached_threads();
  services_available_.store(false);

  // ★ on_shutdown 也执行断电（覆盖 on_deactivate 未被调用的场景）
  emergency_poweroff();

  stop_async_read_thread();

  // 关闭 CAN socket
  {
    std::lock_guard<std::mutex> lock(can_mutex_);
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn on_error(
  const rclcpp_lifecycle::State &) override
{
  services_available_.store(false);
  shutdown_detached_threads();

  // ★ 错误状态下也断电（SDO 可能不可用，走 RPDO）
  send_rpdo1(CMD_QUICK_STOP, 0, MODE_VELOCITY);
  send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);

  stop_async_read_thread();

  // 关闭 CAN socket
  {
    std::lock_guard<std::mutex> lock(can_mutex_);
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

  return CallbackReturn::SUCCESS;
}
```

### 析构函数兜底

```cpp
~IslCHardwareInterface() override
{
  // 最后防线：如果 lifecycle 回调都没执行，析构时尝试断电
  if (can_socket_ >= 0) {
    // 直接发 Quick Stop + Disable Voltage（最短序列，不等延时）
    send_rpdo1(CMD_QUICK_STOP, 0, MODE_VELOCITY);
    send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);

    uint8_t nmt_data[2] = {0x02, node_id_};
    send_can_frame(0x000, nmt_data, 2);

    close(can_socket_);
    can_socket_ = -1;
  }
}
```

### 独立断电节点（备用方案）

当 hardware_interface 的所有回调都未执行时（如 `kill -9`），可用独立节点强制断电：

```bash
# 紧急断电命令
ros2 run motor_poweroff_controller poweroff_node --ros-args \
  -p can_interface:=can0 -p node_id:=1
```

---

## 使用方法

### 1. 集成到现有 hardware_interface

将断电逻辑添加到你的 `SystemInterface` 实现中：

```cpp
// 在你的 hardware_interface 头文件中添加:
private:
  int can_socket_{-1};
  std::mutex can_mutex_;
  uint8_t node_id_{1};
  std::string can_interface_{"can0"};

  bool send_can_frame(uint32_t can_id, const uint8_t* data, uint8_t len);
  bool send_rpdo1(uint16_t controlword, int32_t target_velocity, int8_t mode);
  void emergency_poweroff();
```

### 2. 在 lifecycle 回调中调用

```cpp
CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
{
  // ... 你的清理代码 ...
  emergency_poweroff();  // ★ 添加这一行
  return CallbackReturn::SUCCESS;
}

CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
{
  // ... 你的清理代码 ...
  emergency_poweroff();  // ★ 添加这一行
  return CallbackReturn::SUCCESS;
}

CallbackReturn on_error(const rclcpp_lifecycle::State &) override
{
  // ... 你的清理代码 ...
  send_rpdo1(CMD_QUICK_STOP, 0, MODE_VELOCITY);      // ★ 精简版断电
  send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);
  return CallbackReturn::SUCCESS;
}
```

### 3. 在析构函数中兜底

```cpp
~MyHardwareInterface() override
{
  if (can_socket_ >= 0) {
    send_rpdo1(0x0002, 0, 3);  // Quick Stop
    send_rpdo1(0x0000, 0, 3);  // Disable Voltage
    uint8_t nmt[2] = {0x02, node_id_};
    send_can_frame(0x000, nmt, 2);
    close(can_socket_);
  }
}
```

### 4. 编译独立断电节点（可选）

```bash
cd ~/my_ws
colcon build --packages-select motor_poweroff_controller
source install/setup.bash

# 紧急断电
ros2 run motor_poweroff_controller poweroff_node --ros-args \
  -p can_interface:=can0 -p node_id:=1
```

---

## 断电时序详解

### 正常退出 (Ctrl+C)

```
T+0ms:   SIGINT received
T+5ms:   ROS 2 shutdown initiated
T+10ms:  on_deactivate() called
T+15ms:  shutdown_detached_threads()  ← 停止异步 SDO 读取
T+20ms:  services_available_ = false
T+25ms:  stop_async_read_thread()
T+30ms:  emergency_poweroff() starts
T+31ms:  Step 1: RPDO1 CW=0x010F Vel=0  ← SocketCAN <1ms
T+81ms:  Step 2: RPDO1 CW=0x0002 Vel=0
T+131ms: Step 3: RPDO1 CW=0x0000 Vel=0
T+181ms: Step 4: NMT Stop
T+182ms: close(can_socket_)
T+183ms: on_deactivate() returns SUCCESS
T+200ms: on_cleanup() / on_shutdown() called
T+500ms: Process exits cleanly
```

### 异常退出 (节点崩溃)

```
T+0ms:   Segfault / uncaught exception
T+5ms:   on_error() called (if lifecycle still alive)
T+10ms:  send_rpdo1(QUICK_STOP)      ← 精简版，2步
T+11ms:  send_rpdo1(DISABLE_VOLTAGE)
T+12ms:  close(can_socket_)
T+13ms:  on_error() returns
T+50ms:  Process terminates
         ↓
         若 on_error() 也未执行:
T+0ms:   ~Destructor() called
T+5ms:   send_rpdo1(QUICK_STOP)      ← 析构兜底
T+6ms:   send_rpdo1(DISABLE_VOLTAGE)
T+7ms:   send_can_frame(NMT_STOP)
T+8ms:   close(can_socket_)
```

### 强制杀死 (kill -9)

```
T+0ms:   SIGKILL received
T+0ms:   Process immediately terminated
         ❌ 所有回调都不执行
         ❌ 析构函数不执行
         ⚠️ 电机保持最后状态
         → 需要硬件急停按钮
```

---

## 退出路径覆盖矩阵

| 退出方式 | on_deactivate | on_shutdown | on_error | ~Destructor | 电机断电 |
|---------|:---:|:---:|:---:|:---:|:---:|
| Ctrl+C (正常) | ✅ | ✅ | — | ✅ | ✅ 完整4步 |
| Ctrl+C (ROS忙) | ⚠️ 超时 | ❌ | — | ✅ | ✅ 析构兜底 |
| 节点异常 | — | — | ✅ | ✅ | ✅ 精简2步 |
| segfault | — | — | ⚠️ | ✅ | ✅ 析构兜底 |
| kill -9 | ❌ | ❌ | ❌ | ❌ | ❌ 需硬件急停 |
| ros2 lifecycle set | ✅ | ✅ | — | ✅ | ✅ 完整4步 |

---

## 为什么走 SocketCAN 而非 ROS Service

| 对比项 | ROS Service (SDO) | SocketCAN (RPDO/NMT) |
|--------|-------------------|---------------------|
| 延迟 | 20-500ms | <1ms |
| 依赖 ROS executor | ✅ 需要 | ❌ 不需要 |
| ROS shutdown 后可用 | ❌ 不可用 | ✅ 可用 |
| 4步断电总耗时 | 1-2秒 | ~200ms |
| 失败率 (shutdown 时) | 高（service 已销毁） | 极低（内核级） |
| 代码复杂度 | 需要异步 future/callback | 同步调用，简单直接 |

**关键场景**：`Ctrl+C` 后 ROS 2 开始 shutdown，executor 停止 spin，service server 被销毁。此时通过 ROS Service 发 SDO 请求会直接失败。而 SocketCAN 是 Linux 内核系统调用，与 ROS 生命周期完全无关。

---

## 效果对比

### 修改前

```
^C
[INFO] Shutdown requested
[INFO] on_deactivate: Deactivating...
[ERROR] sdo_write: service not available    ← ROS Service 已销毁
[INFO] Deactivation complete
# 进程退出
# ⚠️ 电机仍在运转！需要手动断电！
```

### 修改后

```
^C
[INFO] Shutdown requested
[INFO] on_deactivate: Deactivating...
[INFO] Sending motor power-off sequence...
[INFO] Step 1: Halt+Enable (CW=0x010F)      ← SocketCAN <1ms
[INFO] Step 2: Quick Stop (CW=0x0002)        ← SocketCAN <1ms
[INFO] Step 3: Disable Voltage (CW=0x0000)   ← SocketCAN <1ms
[INFO] Step 4: NMT Stop                      ← SocketCAN <1ms
[INFO] Motor power-off sequence complete
[INFO] Deactivation complete
# 进程退出
# ✅ 电机已安全停止！
```

---

## 适用范围与限制

### ✅ 适用场景

- CiA402 兼容的伺服电机（雷赛 LD2-CAN、汇川 IS620P、松下 MINAS 等）
- 使用 SocketCAN 的 Linux 系统
- `ros2_control` hardware_interface 架构
- 需要安全断电的任何 CANopen 应用

### ⚠️ 限制

1. **无法处理 SIGKILL**：`kill -9` 直接终止进程，无法执行任何代码。必须依赖硬件急停按钮。
2. **CAN 接口必须在线**：如果 CAN 接口被 `ip link set down`，断电序列会失败。
3. **RPDO 映射前提**：设备的 RPDO1 必须映射了 Controlword (0x6040) + Target Velocity (0x60FF) + Mode of Operation (0x6060)。大部分 CiA402 电机默认如此。
4. **单节点断电**：当前实现一次只断一个节点。多节点系统需循环调用或扩展为多节点版本。

---

## FAQ

### Q: 为什么不直接发送 Disable Voltage (CW=0x0000) 一步到位？

A: 直接 Disable Voltage 会立即切断电机功率，运动的负载可能因惯性飞车。三步序列让电机先按内部减速度平滑停止（Step 1），再用 Quick Stop 兜底（Step 2），最后才 Disable Voltage（Step 3）。

### Q: RPDO 和 SDO 有什么区别？

A:
- **SDO** (Service Data Object)：点对点通信，类似 TCP，有确认/重传机制，延迟 20-500ms。适合配置参数读写。
- **RPDO** (Receive Process Data Object)：主站→从站，类似 UDP广播，无确认，延迟 <1ms。适合实时控制命令。

断电场景下 RPDO 更合适：快、不依赖协议栈状态、不需要确认。

### Q: 如果 RPDO 帧丢了怎么办？

A: 三步序列本身就是冗余设计。如果 Step 1 丢了，Step 2 (Quick Stop) 会兜底。如果 Step 2 也丢了，Step 3 (Disable Voltage) 直接断电。三帧全部丢失的概率极低（同一 CAN 总线连续 3 帧丢失需要总线严重故障）。

### Q: 能否用 `cansend` 命令行工具断电？

A: 可以，但不推荐在程序内部用 `system("cansend ...")`：
1. `system()` 调用 fork+exec，延迟高（10-50ms）
2. 如果 shell 不可用（嵌入式系统）会失败
3. 无法获取发送结果

但可以用于调试或紧急情况：

```bash
# 手动断电序列 (node_id=1)
cansend can0 201#0F01000000000003  # RPDO1: CW=0x010F, Vel=0, Mode=3
sleep 0.05
cansend can0 201#0200000000000003  # RPDO1: CW=0x0002, Vel=0, Mode=3
sleep 0.05
cansend can0 201#0000000000000003  # RPDO1: CW=0x0000, Vel=0, Mode=3
sleep 0.05
cansend can0 000#0201              # NMT Stop, node 1
```

### Q: 为什么 on_error() 只发 2 步而不是 4 步？

A: on_error() 通常在异常状态下被调用，此时系统可能不稳定，需要尽快完成断电。2 步（Quick Stop + Disable Voltage）足以让电机停止，省略了 Halt 平滑减速步骤。如果时间允许 on_error() 也可以调用完整的 `emergency_poweroff()`。

---

## 许可证

Apache 2.0

## 致谢

- [ros2_control](https://github.com/ros-controls/ros2_control) — ROS 2 控制框架
- [ros2_canopen](https://github.com/ros-industrial/ros2_canopen) — ROS 2 CANopen 驱动
- [CiA 402](https://can-cia.org/) — CANopen 驱动配置规范
