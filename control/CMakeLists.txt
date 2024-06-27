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

find_package(OsqpEigen REQUIRED)  #需要找到OSQPeigen

# PID
add_executable(pid_demo PID/main.cpp PID/PID.cpp)
target_link_libraries(pid_demo PRIVATE planning_control_auto)

# PurePursuit
add_executable(pp_demo Pure_pursult/PurePursuit.cpp Pure_pursult/main.cpp)
target_link_libraries(pp_demo PRIVATE planning_control_auto)

# Stanely
add_executable(stanely_demo Stanely/Stanely.cpp Stanely/main.cpp)
target_link_libraries(stanely_demo PRIVATE planning_control_auto)

#lqr
add_executable(lqr_demo LQR/LQR.cpp LQR/main.cpp utils/MyReferencePath.cpp)
target_link_libraries(lqr_demo PRIVATE planning_control_auto)

# mpc
add_executable(mpc_demo MPC/mpc.cpp MPC/main.cpp utils/MyReferencePath.cpp)
target_link_libraries(mpc_demo PRIVATE planning_control_auto OsqpEigen::OsqpEigen) #需要链接到Osqpeigen

#mpc实现轨迹跟踪  利用cppad/ipopt
add_executable(mpc_speed_demo MPC_Speed/mpc_speed.cpp MPC_Speed/main.cpp)
target_link_libraries(mpc_speed_demo PRIVATE planning_control_auto ipopt)