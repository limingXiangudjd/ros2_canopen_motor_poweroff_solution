cmake_minimum_required(VERSION 3.8)
project(motor_poweroff_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 独立断电节点
add_executable(poweroff_node src/poweroff_node.cpp)
target_include_directories(poweroff_node PRIVATE
  include
)
target_link_libraries(poweroff_node)
ament_target_dependencies(poweroff_node rclcpp)

# Hardware Interface 集成示例 (库)
add_library(${PROJECT_NAME} SHARED
  src/motor_poweroff_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

# 插件导出
pluginlib_export_plugin_description_file(hardware_interface poweroff_plugin.xml)

# 安装
install(TARGETS poweroff_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
