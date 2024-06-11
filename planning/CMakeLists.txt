cmake_minimum_required(VERSION 3.21)
project(plannin_control_auto)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# astar
add_executable(astar_demo Astar/Astar.cpp Astar/main.cpp)
target_link_libraries(astar_demo PRIVATE planning_control_auto)

# RRT
add_executable(rrt_demo Rapidly_exploring_Random_tree/main.cpp Rapidly_exploring_Random_tree/RRT.cpp utils/geometry_utils.hpp)
target_link_libraries(rrt_demo PRIVATE planning_control_auto)

#Dubins
add_executable(dubins_demo Dubins_Path/main.cpp Dubins_Path/Dubins.cpp)
target_link_libraries(dubins_demo PRIVATE planning_control_auto)

#RS曲线
add_executable(rs_demo Reeds_Sheep_Path/ReedsSheep.cpp Reeds_Sheep_Path/main.cpp)
target_link_libraries(rs_demo PRIVATE planning_control_auto)

##DWA
add_executable(dwa_demo Dynamic_Window_Approach/DWA.cpp Dynamic_Window_Approach/main.cpp)
target_link_libraries(dwa_demo PRIVATE planning_control_auto)

#Bezier
add_executable(bezier_demo Bezier/BezierCurve.cpp Bezier/main.cpp)
target_link_libraries(bezier_demo PRIVATE planning_control_auto)

#BSpline
add_executable(b_spline_demo BSpline/BSpline.cpp BSpline/main.cpp)
target_link_libraries(b_spline_demo PRIVATE planning_control_auto)

#APF
add_executable(apf_demo Artifical_Potential_FIeld/APF.cpp Artifical_Potential_FIeld/main.cpp)
target_link_libraries(apf_demo PRIVATE planning_control_auto)