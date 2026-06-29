# 项目结构

```
ros2_canopen_motor_poweroff_solution/
├── README.md                          # 主文档（问题背景+方案+时序+FAQ）
│
├── src/motor_poweroff_controller/     # ROS 2 包
│   ├── CMakeLists.txt                 # 构建配置
│   ├── package.xml                    # 包描述
│   ├── poweroff_plugin.xml            # pluginlib 插件描述
│   ├── include/motor_poweroff_controller/
│   │   └── motor_poweroff_controller.hpp   # 核心头文件（SocketCAN 断电逻辑）
│   ├── src/
│   │   ├── motor_poweroff_controller.cpp   # Hardware Interface 集成示例
│   │   └── poweroff_node.cpp               # 独立断电节点
│   ├── config/
│   │   └── poweroff_config.yaml       # 节点配置
│   └── launch/
│       └── poweroff.launch.py         # launch 文件
│
├── docs/
│   ├── CIA402_POWEROFF_TECHNICAL.md   # CiA402 断电技术说明
│   └── TROUBLESHOOTING.md             # 故障排查
│
└── examples/
    ├── example.urdf.xacro             # URDF 配置示例
    ├── ros2_controllers.yaml.example  # 控制器配置示例
    └── emergency_poweroff.sh          # Bash 手动断电脚本
```

## 快速开始

### 方式 1: 集成到现有 hardware_interface

1. 复制 `motor_poweroff_controller.hpp` 中的断电逻辑到你的 SystemInterface 类
2. 在 `on_deactivate()` / `on_shutdown()` 中调用 `emergency_poweroff()`
3. 在 `on_error()` 中调用 `quick_poweroff()`
4. 在析构函数中添加兜底断电
5. 参考 `motor_poweroff_controller.cpp` 中的集成示例

### 方式 2: 使用独立断电节点

```bash
# 编译
cd ~/my_ws
colcon build --packages-select motor_poweroff_controller
source install/setup.bash

# 紧急断电
ros2 launch motor_poweroff_controller poweroff.launch.py can_interface:=can0 node_id:=1
```

### 方式 3: 用 Bash 脚本手动断电

```bash
chmod +x examples/emergency_poweroff.sh
./examples/emergency_poweroff.sh can0 1
```

## 核心文件说明

| 文件 | 作用 |
|------|------|
| `motor_poweroff_controller.hpp` | 可复用的断电逻辑（SocketCAN + RPDO1 + NMT） |
| `motor_poweroff_controller.cpp` | 集成到 SystemInterface 的完整示例 |
| `poweroff_node.cpp` | 独立 ROS 2 节点，运行后立即断电并退出 |
| `emergency_poweroff.sh` | Bash 脚本，用 `cansend` 手动断电 |
