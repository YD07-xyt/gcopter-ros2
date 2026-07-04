cmake_minimum_required(VERSION 3.8)
project(gcopter)

# 使用 C++14 标准（ROS2 默认支持 C++14/17）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

# 查找依赖
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(ompl REQUIRED)
find_package(spdlog REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
# ROS2 核心依赖
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# CasADi 通常安装在 /usr/local
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# 包含头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(planner
src/omni_mpc.cpp  src/ros2.cpp
src/ros2_2d.cpp
src/lmpc.cpp
src/lmpc_tracker.cpp)

target_link_libraries(planner
  ${OMPL_LIBRARIES}
  spdlog::spdlog
  casadi
  osqp::osqp
  Boost::boost
  OsqpEigen::OsqpEigen
)

ament_target_dependencies(planner
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
)

# 添加可执行文件
add_executable(global_planning src/global_planning.cpp)

# 链接依赖库
target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
  spdlog::spdlog
  casadi
  OsqpEigen::OsqpEigen
  Boost::boost
  libqpOASES.a
  planner
)

# 使用 ament_target_dependencies 添加 ROS2 依赖
target_link_libraries(global_planning
  ${EIGEN3_LIBRARIES}
)

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