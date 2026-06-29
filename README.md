# ros2_canopen_motor_poweroff_solution
 A safety-critical solution for ros2_control + ros2_canopen setups where motors fail to power down upon node exit (e.g., Ctrl+C). By bypassing the unreliable ROS Service communication during shutdown, this package uses Linux Kernel-level SocketCAN to directly send CiA402 power-off sequences (RPDO + NMT).
