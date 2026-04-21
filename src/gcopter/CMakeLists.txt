cmake_minimum_required(VERSION 3.8)
project(gcopter)

# 使用 C++14 标准（ROS2 默认支持 C++14/17）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

# 查找依赖
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

# ROS2 核心依赖
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)



add_subdirectory(fastdem)
# 包含头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(global_planning src/global_planning.cpp)

# 链接依赖库
target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
   fastdem
   ${EIGEN3_LIBRARIES}
)

# 使用 ament_target_dependencies 添加 ROS2 依赖
ament_target_dependencies(global_planning
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
)

# 安装规则
install(TARGETS global_planning
  DESTINATION lib/${PROJECT_NAME})

# 导出包
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)