// Copyright 2024
// Licensed under the Apache License, Version 2.0

/**
 * Hardware Interface 集成示例
 *
 * 展示如何将断电逻辑集成到 ros2_control SystemInterface 中。
 * 本文件是模板代码，不能直接编译——需要结合你的实际硬件接口实现。
 *
 * 关键集成点:
 *   1. on_init: 读取 can_interface 和 node_id 参数
 *   2. on_deactivate: 调用 emergency_poweroff()
 *   3. on_shutdown: 调用 emergency_poweroff()
 *   4. on_error: 调用 quick_poweroff()
 *   5. ~析构函数: 兜底断电
 */

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motor_poweroff_controller/motor_poweroff_controller.hpp"

namespace my_hardware_interface
{

class ExampleHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ExampleHardwareInterface)

  ~ExampleHardwareInterface() override
  {
    // ★ 析构兜底：最后防线
    if (poweroff_controller_) {
      poweroff_controller_->quick_poweroff();
      poweroff_controller_->close_socket();
    }
  }

  // ========================================================================
  // Lifecycle 回调
  // ========================================================================

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    // 读取 CAN 接口和节点 ID 参数
    // 从 URDF <hardware> <param> 标签中读取
    can_interface_ = "can0";  // 默认值
    node_id_ = 1;             // 默认值

    for (const auto & param : info.hardware_parameters) {
      if (param.first == "can_interface") {
        can_interface_ = param.second;
      } else if (param.first == "node_id") {
        node_id_ = std::stoul(param.second);
      }
    }

    // 初始化断电控制器
    poweroff_controller_ = std::make_unique<MotorPoweroffController>();
    if (!poweroff_controller_->init(can_interface_, static_cast<uint8_t>(node_id_))) {
      RCLCPP_ERROR(rclcpp::get_logger("ExampleHardwareInterface"),
        "Failed to init CAN socket on '%s'", can_interface_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // 你的配置代码...
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // 你的激活代码...
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("ExampleHardwareInterface"), "Deactivating...");

    // 1. 停止异步线程
    // stop_async_threads();

    // 2. ★ 执行断电序列 (SocketCAN 直发，不依赖 ROS Service)
    if (poweroff_controller_) {
      poweroff_controller_->emergency_poweroff();
    }

    RCLCPP_INFO(rclcpp::get_logger("ExampleHardwareInterface"), "Deactivation complete");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // 清理资源...
    if (poweroff_controller_) {
      poweroff_controller_->close_socket();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("ExampleHardwareInterface"), "Shutting down...");

    // ★ on_shutdown 也断电（覆盖 on_deactivate 未被调用的场景）
    if (poweroff_controller_) {
      poweroff_controller_->emergency_poweroff();
      poweroff_controller_->close_socket();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_ERROR(rclcpp::get_logger("ExampleHardwareInterface"), "Error state!");

    // ★ 错误状态：精简版断电 (2步)
    if (poweroff_controller_) {
      poweroff_controller_->quick_poweroff();
      poweroff_controller_->close_socket();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ========================================================================
  // 接口导出
  // ========================================================================

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    // 返回状态接口...
    return {};
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    // 返回命令接口...
    return {};
  }

  // ========================================================================
  // 读写循环
  // ========================================================================

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // 读取状态...
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // 写入命令...
    return hardware_interface::return_type::OK;
  }

private:
  std::string can_interface_{"can0"};
  uint8_t node_id_{1};
  std::unique_ptr<MotorPoweroffController> poweroff_controller_;
};

}  // namespace my_hardware_interface

// 导出插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_hardware_interface::ExampleHardwareInterface,
  hardware_interface::SystemInterface
)
