cmake_minimum_required(VERSION 3.10)
# 行为树节点。patrol.xml 安装到 share，运行时按包名查找。
project(sitl_mission_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sitl_adapters REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/bt_nodes.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  behaviortree_cpp_v3
  sitl_adapters
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
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
  behaviortree_cpp_v3
  sitl_adapters
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(bt_patrol_test test/bt_patrol_test.cpp)
  target_include_directories(bt_patrol_test PUBLIC include)
  ament_target_dependencies(bt_patrol_test rclcpp behaviortree_cpp_v3 sitl_adapters)
  target_link_libraries(bt_patrol_test ${PROJECT_NAME})
endif()

ament_package()
