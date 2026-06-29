# ROS 2 CANopen Motor Safe Poweroff Solution
# ROS 2 CANopen 电机安全断电方案
> **English Summary**: A safety-critical solution for `ros2_control` + `ros2_canopen` setups where motors fail to power down upon node exit (e.g., Ctrl+C). By bypassing the unreliable ROS Service communication during shutdown, this package uses Linux Kernel-level SocketCAN to directly send CiA402 power-off sequences (RPDO + NMT). It guarantees motor stoppage in any exit path (lifecycle callbacks, errors, or destructors) within ~200ms.
> **中文摘要**：针对 `ros2_control` + `ros2_canopen` 程序退出（如 Ctrl+C）后电机不掉电的安全隐患，本方案绕过不可靠的 ROS Service 通信，通过 Linux 内核级 SocketCAN 直接发送 CiA402 断电指令序列（RPDO + NMT）。确保在 Lifecycle 回调、异常错误或析构函数等任何退出路径下，电机均能在 200ms 内安全停止。
---
## 📋 目录
- [问题背景](#-问题背景)
- [根因分析](#-根因分析)
- [解决方案架构](#-解决方案架构)
- [核心实现](#-核心实现)
- [使用方法](#-使用方法)
- [效果对比](#-效果对比)
- [适用范围与限制](#-适用范围与限制)
- [FAQ](#-faq)
---
## 📋 问题背景
### English Description
In ROS 2 Humble, when stopping a `ros2_control` node with `Ctrl+C`, the process exits cleanly, but **the servo motor continues running** at the last commanded velocity.
- **Symptom**: Motor remains energized and moving after node shutdown.
- **Hazard**: Equipment runaway, potential mechanical damage, or safety risks if E-stop fails due to CAN bus occupation.
### 中文描述
在 ROS 2 Humble 中使用 `ros2_control` 驱动伺服电机时，`Ctrl+C` 停止程序后：
- **现象**：程序正常退出，无崩溃，但**电机仍在运转**，保持最后一个速度命令。
- **危害**：设备失控移动，可能损坏机械结构，若硬件急停按钮被阻塞将造成安全事故。
---
## 🔍 根因分析
### English Description
ROS 2 Lifecycle callbacks (`on_deactivate`, `on_shutdown`) are not guaranteed to execute fully during a SIGINT (Ctrl+C) shutdown.
- **Time Constraint**: ROS 2 forces termination after a short timeout, interrupting the cleanup process.
- **ROS Service Unavailable**: During shutdown, the executor and service servers are destroyed. Attempts to call `sdo_write` services for power-off commands fail instantly.
- **SDO Latency**: Standard SDO communication takes 20-500ms per request. A 4-step power-off sequence (1-2 seconds) cannot complete before the process is killed.
### 中文描述
ROS 2 的 Lifecycle 回调在 SIGINT (Ctrl+C) 信号下的执行具有不确定性。
- **时间紧迫**：ROS 2 shutdown 流程有严格超时限制，断电序列可能未完成即被强制终止。
- **ROS Service 失效**：Shutdown 过程中 executor/service server 被销毁，调用 ROS Service 发送 SDO 指令会直接失败。
- **SDO 响应慢**：单个 SDO 请求耗时 20-500ms，完整断电需 1-2 秒，来不及发送。
---
## 🛠️ 解决方案架构
### English Description
We replace ROS Service calls with **Linux Kernel-level SocketCAN**.
- **Mechanism**: Directly writing CAN frames to the socket (`write()`) bypasses all ROS infrastructure. It works as long as the process is alive and the CAN interface is up.
- **Speed**: Frame transmission takes <1ms. Total power-off sequence: ~200ms.
- **Defense Lines**:
    1. `on_deactivate()` / `on_shutdown()` (Normal Exit).
    2. `on_error()` (Exception Exit).
    3. `~Destructor()` (Final Fallback).
    4. Standalone Node / Bash Script (Manual Fallback).
### 中文描述
我们采用 **Linux 内核级 SocketCAN** 替代 ROS Service 调用。
- **机制**：直接向 Socket 写入 CAN 帧，绕过所有 ROS 中间件，只要进程存活即可发送。
- **速度**：单帧发送 <1ms，完整断电序列仅需 ~200ms。
- **三层防线**：
    1. `on_deactivate()` / `on_shutdown()`（正常退出）。
    2. `on_error()`（异常退出）。
    3. `~析构函数()`（最后兜底）。
    4. 独立节点 / Bash 脚本（手动备用）。
---
## 💻 核心实现
### CiA402 Power-Off Sequence / CiA402 断电序列
| Step | English Action | 中文动作 | Controlword (CW) | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **1** | Halt + Enable | 减速停止 | `0x010F` | Smooth deceleration to 0. |
| **2** | Quick Stop | 快速停止 | `0x0002` | Aggressive stop if Step 1 fails. |
| **3** | Disable Voltage | 关闭使能 | `0x0000` | Cut motor power output. |
| **4** | NMT Stop | 停止通信 | `0x000#0201` | Stop all PDO communication. |
### Key Code: SocketCAN Direct Send / 关键代码：SocketCAN 直发
**English**: Uses `write()` syscall to send RPDO1 frames. No ROS dependency.
**中文**：使用 `write()` 系统调用发送 RPDO1 帧，无 ROS 依赖。
```cpp
// motor_poweroff_controller.hpp
void emergency_poweroff() {
    // Step 1: Halt + Enable (CW=0x010F)
    send_rpdo1(0x010F, 0, 3); 
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Step 2: Quick Stop (CW=0x0002)
    send_rpdo1(0x0002, 0, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Step 3: Disable Voltage (CW=0x0000)
    send_rpdo1(0x0000, 0, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Step 4: NMT Stop
    uint8_t nmt_data[2] = {0x02, node_id_};
    send_can_frame(0x000, nmt_data, 2);
}
📂 文件清单
🚀 使用方法
Method 1: Integrate into Hardware Interface / 方式 1: 集成到 Hardware Interface
English: Add emergency_poweroff() calls to your lifecycle callbacks.
中文：在 Lifecycle 回调中添加 emergency_poweroff() 调用。
// Your Hardware Interface Implementation
CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    emergency_poweroff(); // ★ Call here
    return CallbackReturn::SUCCESS;
}
~MyHardwareInterface() {
    // Destructor Fallback
    if (can_socket_ >= 0) {
        send_rpdo1(0x0002, 0, 3); // Quick Stop
        send_rpdo1(0x0000, 0, 3); // Disable
        close(can_socket_);
    }
}
Method 2: Standalone Node / 方式 2: 独立断电节点
# Build / 编译
colcon build --packages-select motor_poweroff_controller
source install/setup.bash
# Emergency Poweroff / 紧急断电
ros2 run motor_poweroff_controller poweroff_node --ros-args \
  -p can_interface:=can0 -p node_id:=1
📊 效果对比
English Description
Before: Motor continues running after Ctrl+C. Requires manual E-stop.
After: Motor safely stops in ~200ms. Process exits cleanly.
中文描述
修改前：Ctrl+C 后电机持续运转，需手动急停。
修改后：Ctrl+C 后电机在 200ms 内安全停止，进程正常退出。
⚠️ 适用范围与限制
Applicable / 适用
CiA402 compliant servo motors (Leisai LD2-CAN, etc.).
Linux systems with SocketCAN support.
ROS 2 Humble / ros2_control architecture.
Limitations / 限制
SIGKILL (kill -9): Cannot stop the motor as no code executes. Requires physical E-stop button.
RPDO Mapping: Device RPDO1 must be mapped to CW + Velocity + Mode (standard for most CiA402 drives).
CAN Interface: Socket must remain open; fails if interface is down.
🔍 FAQ
Q: Why SocketCAN instead of ROS Service?
问：为什么用 SocketCAN 而非 ROS Service？
A: ROS Service relies on the executor and DDS middleware, which are destroyed during shutdown. SocketCAN is a kernel syscall that works independently of ROS state, ensuring the command gets through even if the node is dying.
答：ROS Service 依赖 Executor 和 DDS 中间件，在 Shutdown 时被销毁。SocketCAN 是内核级系统调用，独立于 ROS 状态，即使节点正在退出也能保证指令送达。
Q: Why 4 steps instead of just "Disable Voltage"?
问：为什么要 4 步而不是直接 "Disable Voltage"？
A: Abruptly cutting power (Disable Voltage) on a moving load may cause mechanical damage due to inertia. The sequence (Halt -> Quick Stop -> Disable) ensures controlled deceleration before power cut.
答：直接切断功率会让惯性负载突然停止，可能损坏机械。序列（减速停止 -> 快速停止 -> 断电）确保在切断电源前先完成受控减速。
Q: What if kill -9 happens?
问：如果遇到 kill -9 怎么办？
A: No software can handle kill -9. A physical Hardware Emergency Stop (E-stop) button is mandatory as the final safety layer.
答：没有任何软件能处理 kill -9。必须配置物理硬件急停按钮作为最后一道安全防线。
📚 引用
If you use this solution, please refer to the CiA 402 standard and ros2_control documentation.
如果您使用本方案，请参考 CiA 402 标准及 ros2_control 文档。
License: Apache 2.0
Acknowledgement: ros2_control, ros2_canopen, CiA 402 Standard.
