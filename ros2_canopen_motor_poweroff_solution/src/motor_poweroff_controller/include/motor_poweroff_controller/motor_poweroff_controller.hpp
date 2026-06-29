// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef MOTOR_POWEROFF_CONTROLLER__MOTOR_POWEROFF_CONTROLLER_HPP_
#define MOTOR_POWEROFF_CONTROLLER__MOTOR_POWEROFF_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace motor_poweroff_controller
{

/**
 * @brief CiA402 电机安全断电 Hardware Interface 模板
 *
 * 本文件提供可直接集成到 ros2_control SystemInterface 的断电逻辑。
 * 核心能力:
 *   - SocketCAN 直接发送 CAN 帧 (不依赖 ROS Service)
 *   - 三步渐进断电序列 (Halt → Quick Stop → Disable Voltage → NMT Stop)
 *   - 4 个 lifecycle 回调全覆盖 (deactivate/cleanup/shutdown/error)
 *   - 析构函数兜底
 *
 * 集成方式:
 *   1. 将本文件中的 private 方法和成员变量复制到你的 SystemInterface 类
 *   2. 在 on_deactivate/on_shutdown/on_error 中调用 emergency_poweroff()
 *   3. 在析构函数中添加兜底断电
 *   4. 在 on_init 中读取 can_interface 和 node_id 参数
 */

// ============================================================================
// CiA402 常量定义 (可放入头文件供多个类共享)
// ============================================================================

namespace cia402
{

// 对象字典索引
static constexpr uint16_t OBJ_CONTROLWORD = 0x6040;
static constexpr uint16_t OBJ_STATUSWORD = 0x6041;
static constexpr uint16_t OBJ_TARGET_VELOCITY = 0x60FF;
static constexpr uint16_t OBJ_MODE_OF_OPERATION = 0x6060;

// 控制字命令
static constexpr uint16_t CMD_SHUTDOWN = 0x0006;
static constexpr uint16_t CMD_SWITCH_ON = 0x0007;
static constexpr uint16_t CMD_ENABLE_OPERATION = 0x000F;
static constexpr uint16_t CMD_DISABLE_VOLTAGE = 0x0000;
static constexpr uint16_t CMD_QUICK_STOP = 0x0002;
static constexpr uint16_t CMD_FAULT_RESET = 0x0080;
static constexpr uint16_t CMD_HALT = 0x0100;  // Bit 8 = Halt

// 操作模式
static constexpr int8_t MODE_NO_MODE = 0;
static constexpr int8_t MODE_POSITION = 1;
static constexpr int8_t MODE_VELOCITY = 3;
static constexpr int8_t MODE_HOMING = 6;

}  // namespace cia402


/**
 * @brief 独立的安全断电控制器
 *
 * 可作为独立节点运行，也可嵌入到 SystemInterface 中。
 * 通过 SocketCAN 直接发送 CiA402 断电指令序列。
 */
class MotorPoweroffController
{
public:
  MotorPoweroffController() = default;
  ~MotorPoweroffController()
  {
    // 析构兜底：如果 socket 仍开着，尝试断电
    if (can_socket_ >= 0) {
      // 精简版断电（不等延时）
      send_rpdo1(cia402::CMD_QUICK_STOP, 0, cia402::MODE_VELOCITY);
      send_rpdo1(cia402::CMD_DISABLE_VOLTAGE, 0, cia402::MODE_VELOCITY);
      uint8_t nmt_data[2] = {0x02, node_id_};
      send_can_frame(0x000, nmt_data, 2);
      close(can_socket_);
      can_socket_ = -1;
    }
  }

  /**
   * @brief 初始化 CAN 接口
   * @param can_interface CAN 接口名称 (如 "can0")
   * @param node_id CANopen 节点 ID
   * @return 成功返回 true
   */
  bool init(const std::string & can_interface, uint8_t node_id)
  {
    can_interface_ = can_interface;
    node_id_ = node_id;
    return open_can_socket();
  }

  /**
   * @brief 三步渐进断电序列
   *
   * Step 1: Halt + Enable (CW=0x010F, Vel=0)  — 减速停止
   * Step 2: Quick Stop (CW=0x0002, Vel=0)      — 快速停止
   * Step 3: Disable Voltage (CW=0x0000, Vel=0)  — 关闭使能
   * Step 4: NMT Stop                            — 停止所有通信
   *
   * 总耗时 ~200ms，走 SocketCAN 直接发送
   */
  void emergency_poweroff()
  {
    RCLCPP_INFO(rclcpp::get_logger("MotorPoweroffController"),
      "Starting emergency power-off sequence...");

    // Step 1: Halt + Enable Operation
    // CW = 0x010F = Enable(0x000F) | Halt(0x0100)
    // 电机使用内部减速度平滑减速到 0
    send_rpdo1(cia402::CMD_ENABLE_OPERATION | cia402::CMD_HALT, 0, cia402::MODE_VELOCITY);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Step 2: Quick Stop
    // CW = 0x0002, 触发 CiA402 Quick Stop 功能
    // 使用 quick_stop_option_code (对象字典 0x605A) 配置的减速方式
    send_rpdo1(cia402::CMD_QUICK_STOP, 0, cia402::MODE_VELOCITY);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Step 3: Disable Voltage
    // CW = 0x0000, 关闭电机功率输出
    send_rpdo1(cia402::CMD_DISABLE_VOLTAGE, 0, cia402::MODE_VELOCITY);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Step 4: NMT Stop
    // COB-ID = 0x000, data = [0x02, node_id]
    // 使设备进入 Stopped 状态，停止所有 PDO 通信
    uint8_t nmt_data[2] = {0x02, node_id_};
    send_can_frame(0x000, nmt_data, 2);

    RCLCPP_INFO(rclcpp::get_logger("MotorPoweroffController"),
      "Emergency power-off complete (Halt -> QuickStop -> Disable -> NMT Stop)");
  }

  /**
   * @brief 精简版断电 (2步，用于 on_error 等时间紧迫场景)
   */
  void quick_poweroff()
  {
    send_rpdo1(cia402::CMD_QUICK_STOP, 0, cia402::MODE_VELOCITY);
    send_rpdo1(cia402::CMD_DISABLE_VOLTAGE, 0, cia402::MODE_VELOCITY);

    uint8_t nmt_data[2] = {0x02, node_id_};
    send_can_frame(0x000, nmt_data, 2);
  }

  /**
   * @brief 关闭 CAN socket
   */
  void close_socket()
  {
    std::lock_guard<std::mutex> lock(can_mutex_);
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

private:
  /**
   * @brief 打开 CAN socket
   */
  bool open_can_socket()
  {
    std::lock_guard<std::mutex> lock(can_mutex_);

    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MotorPoweroffController"),
        "Failed to create CAN socket: %s", strerror(errno));
      return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MotorPoweroffController"),
        "Failed to get CAN interface index for '%s': %s",
        can_interface_.c_str(), strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MotorPoweroffController"),
        "Failed to bind CAN socket: %s", strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("MotorPoweroffController"),
      "CAN socket opened on '%s' for node %d", can_interface_.c_str(), node_id_);
    return true;
  }

  /**
   * @brief 通过 SocketCAN 直接发送 CAN 帧
   *
   * 不依赖 ROS 通信基础设施，只要进程活着就能发。
   */
  bool send_can_frame(uint32_t can_id, const uint8_t * data, uint8_t len)
  {
    if (len > 8) return false;

    std::lock_guard<std::mutex> lock(can_mutex_);

    // 如果 socket 未开，尝试打开
    if (can_socket_ < 0) {
      // 临时解锁以调用 open_can_socket
      // (open_can_socket 内部会加锁)
      // 这里采用惰性初始化模式
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

    ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
      RCLCPP_ERROR(rclcpp::get_logger("MotorPoweroffController"),
        "Failed to send CAN frame (ID=0x%X): %s", can_id, strerror(errno));
      return false;
    }

    return true;
  }

  /**
   * @brief 发送 RPDO1 帧
   *
   * RPDO1 COB-ID = 0x200 + node_id
   * 帧格式: [CW_lo, CW_hi, Vel_b0, Vel_b1, Vel_b2, Vel_b3, Mode] = 7 bytes
   */
  bool send_rpdo1(uint16_t controlword, int32_t target_velocity, int8_t mode)
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

  // =========================================================================
  // 成员变量
  // =========================================================================

  std::string can_interface_{"can0"};
  uint8_t node_id_{1};
  int can_socket_{-1};
  std::mutex can_mutex_;
};

#endif  // MOTOR_POWEROFF_CONTROLLER__MOTOR_POWEROFF_CONTROLLER_HPP_
