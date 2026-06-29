// Copyright 2024
// Licensed under the Apache License, Version 2.0

/**
 * 独立断电节点
 *
 * 用途: 当 ros2_control 节点异常退出后电机仍在运转时，
 *       运行此节点强制断电。
 *
 * 用法:
 *   ros2 run motor_poweroff_controller poweroff_node --ros-args \
 *     -p can_interface:=can0 -p node_id:=1
 *
 * 或在 launch 文件中:
 *   ros2 launch motor_poweroff_controller poweroff.launch.py
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "motor_poweroff_controller/motor_poweroff_controller.hpp"

class PoweroffNode : public rclcpp::Node
{
public:
  PoweroffNode() : rclcpp::Node("motor_poweroff_node")
  {
    // 声明参数
    can_interface_ = this->declare_parameter("can_interface", "can0");
    node_id_ = this->declare_parameter("node_id", 1);

    RCLCPP_INFO(this->get_logger(),
      "Motor poweroff node starting: interface=%s, node_id=%d",
      can_interface_.c_str(), node_id_);

    // 初始化断电控制器
    if (!controller_.init(can_interface_, static_cast<uint8_t>(node_id_))) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init CAN socket, exiting");
      rclcpp::shutdown();
      return;
    }

    // 立即执行断电
    RCLCPP_INFO(this->get_logger(), "Executing emergency power-off...");
    controller_.emergency_poweroff();

    // 等待断电完成
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    RCLCPP_INFO(this->get_logger(), "Power-off complete, closing socket...");
    controller_.close_socket();

    // 退出节点
    rclcpp::shutdown();
  }

  ~PoweroffNode()
  {
    controller_.close_socket();
  }

private:
  std::string can_interface_;
  int node_id_{1};
  MotorPoweroffController controller_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoweroffNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
