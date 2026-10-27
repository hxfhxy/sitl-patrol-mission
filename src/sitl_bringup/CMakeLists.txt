cmake_minimum_required(VERSION 3.10)
# 只有 launch、参数和模型，没有 C++。
project(sitl_bringup)

find_package(ament_cmake REQUIRED)

# 把 launch/、config/、models/ 目录安装到 share/sitl_bringup/
install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()