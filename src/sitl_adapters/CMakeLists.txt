cmake_minimum_required(VERSION 3.10)
project(sitl_adapters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/nav_adapter.cpp
  src/sensor_health.cpp
  src/estop.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  sensor_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(scan_freshness_test test/scan_freshness_test.cpp)
  target_include_directories(scan_freshness_test PUBLIC include)
  ament_target_dependencies(scan_freshness_test rclcpp sensor_msgs)

  ament_add_gtest(goal_slot_test test/goal_slot_test.cpp)
  target_include_directories(goal_slot_test PUBLIC include)

  ament_add_gtest(sensor_status_test test/sensor_status_test.cpp)
  target_include_directories(sensor_status_test PUBLIC include)
  target_link_libraries(sensor_status_test ${PROJECT_NAME})
  ament_target_dependencies(sensor_status_test rclcpp sensor_msgs)

  ament_add_gtest(estop_gate_test test/estop_gate_test.cpp)
  target_include_directories(estop_gate_test PUBLIC include)
  target_link_libraries(estop_gate_test ${PROJECT_NAME})
  ament_target_dependencies(estop_gate_test rclcpp geometry_msgs)
endif()

ament_package()