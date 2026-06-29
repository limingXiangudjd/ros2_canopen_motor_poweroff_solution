# 故障排查指南

## 问题 1: 断电序列发送后电机仍在运转

### 排查步骤

1. **检查 CAN socket 是否打开成功**
   ```
   [ERROR] Failed to create CAN socket: ...
   ```
   → 检查 CAN 接口是否存在: `ip link show can0`
   → 检查权限: `sudo setcap cap_net_raw+ep $(which device_container_node)`

2. **检查 RPDO 映射**
   RPDO1 必须映射了 Controlword (0x6040) + Target Velocity (0x60FF) + Mode (0x6060)。
   通过 EDS 文件或 `canopen-tool` 检查设备 RPDO 配置。

3. **用 candump 验证 CAN 帧是否发出**
   ```bash
   # 终端1: 监听 CAN 总线
   candump can0

   # 终端2: 触发断电
   ros2 run motor_poweroff_controller poweroff_node --ros-args -p can_interface:=can0 -p node_id:=1

   # 应看到 4 帧:
   # can0  201   [7]  0F 01 00 00 00 00 03    ← RPDO1: CW=0x010F
   # can0  201   [7]  02 00 00 00 00 00 03    ← RPDO1: CW=0x0002
   # can0  201   [7]  00 00 00 00 00 00 03    ← RPDO1: CW=0x0000
   # can0  000   [2]  02 01                    ← NMT Stop
   ```

4. **检查节点 ID 是否正确**
   RPDO1 COB-ID = 0x200 + node_id。如果节点 ID 不对，CAN 帧会被设备忽略。

## 问题 2: on_deactivate 执行了但没断电

### 可能原因

1. **ROS Service 调用阻塞了 on_deactivate**
   → 确保断电代码走 SocketCAN（`send_can_frame`），不走 SDO Service
   → 确保在断电前停止异步读取线程（否则 SDO 超时会阻塞）

2. **detached thread 仍在运行**
   → 确保在断电前调用 `shutdown_detached_threads()`
   → 否则 detached thread 可能访问已销毁的资源导致 crash

## 问题 3: Ctrl+C 后 on_deactivate 没被调用

### 可能原因

1. **lifecycle manager 未配置**
   → 确保 launch 文件中使用了 `ros2_control_node`，它会管理 lifecycle
   → 或手动管理: `ros2 lifecycle set /hardware_component deactivate`

2. **ROS 2 shutdown 超时**
   → ROS 2 给 shutdown 流程有限时间，超时后强制杀死
   → 确保断电序列 < 500ms（当前设计 ~200ms）

## 问题 4: 析构函数没执行

### 可能原因

1. **进程被 SIGKILL 杀死**
   → `kill -9` 不执行析构函数，只能依赖硬件急停

2. **进程 segfault**
   → segfault 后进程直接终止，析构函数可能不执行
   → 这就是为什么需要在 on_error() 中也调用断电

## 调试技巧

### 用 cansend 手动测试断电序列

```bash
# node_id=1, RPDO1 COB-ID=0x201
# CW=0x010F (Halt+Enable), Vel=0, Mode=3(Velocity)
cansend can0 201#0F01000000000003
sleep 0.05

# CW=0x0002 (Quick Stop)
cansend can0 201#0200000000000003
sleep 0.05

# CW=0x0000 (Disable Voltage)
cansend can0 201#0000000000000003
sleep 0.05

# NMT Stop
cansend can0 000#0201
```

### 查看电机状态字

```bash
# 读 Statusword (0x6041)
# SDO Read 请求: COB-ID=0x601, data=[40 41 60 00 00 00 00 00]
cansend can0 601#4041600000000000
candump can0 -n 1  # 等待回复: 581#434160000000XXXX
# XXXX 的 bit 解读:
#   bit0 (Ready to switch on)
#   bit1 (Switched on)
#   bit2 (Operation enabled)  ← 断电后应为 0
#   bit3 (Fault)
#   bit5 (Quick stop active)
#   bit6 (Switch on disabled) ← 断电后应为 1
```
