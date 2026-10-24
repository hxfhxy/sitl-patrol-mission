cmake_minimum_required(VERSION 3.10)
project(sitl_mission_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sitl_adapters REQUIRED)
find_package(sitl_mission_bt REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(mission_node
  src/mission_node.cpp
  src/mqtt_task_client.cpp
)
target_include_directories(mission_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_node
  rclcpp
  std_msgs
  sitl_adapters
  sitl_mission_bt
  behaviortree_cpp_v3
  ament_index_cpp
  lifecycle_msgs
)

# 独立的 cmd_vel 周期探针
add_executable(cmd_vel_probe src/cmd_vel_probe.cpp)
ament_target_dependencies(cmd_vel_probe
  rclcpp
  geometry_msgs
)

# 独立的激光雷达延迟注入节点
add_executable(scan_delay_node src/scan_delay_node.cpp)
ament_target_dependencies(scan_delay_node
  rclcpp
  sensor_msgs
)

# 独立的 odom -> base_footprint TF 门控转发节点
add_executable(odom_tf_gate src/odom_tf_gate.cpp)
ament_target_dependencies(odom_tf_gate
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS mission_node cmd_vel_probe scan_delay_node odom_tf_gate
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(task_command_test test/task_command_test.cpp)
  target_include_directories(task_command_test PUBLIC include)

  ament_add_gtest(mqtt_roundtrip_test test/mqtt_roundtrip_test.cpp src/mqtt_task_client.cpp)
  target_include_directories(mqtt_roundtrip_test PUBLIC include)
endif()

ament_package()