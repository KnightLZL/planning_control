# 最小版本限制
cmake_minimum_required(VERSION 3.21)
# 设置项目名
project(planning_control_auto)

# 设置编译规则
set(CMAKE_CXX_STANDARD 11)

# 在系统查找并加载EIGEN3 库
find_package(Eigen3 REQUIRED)
# 将找到的EIGEN3头文件目录添加到编译器的包含路径中
include_directories(${EIGEN3_INCLUDE_DIR})

# PID
add_executable(pid_demo PID/main.cpp PID/PID.cpp utils/KinematicModel.hpp)
target_link_libraries(pid_demo PRIVATE planning_control_auto)


