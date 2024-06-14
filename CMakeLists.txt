cmake_minimum_required(VERSION 3.21)
project(planning_control_auto)

set(CMAKE_CXX_STANDARD 14) #设置编译规则

# 添加eigens库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(PACKAGE_NAME planning_control_auto)

# 添加链接库
add_library(planning_control_auto INTERFACE)

# 添加外部连接
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(planning_control_auto INTERFACE
        Python3::Python
        Python3::Module
        )
find_package(Python3 COMPONENTS Numpy)
if(Python3_Numpy_FOUND)
    target_link_libraries(planning_control_auto INTERFACE
            Python3::Numpy
            )
else()
    target_compile_definitions(planning_control_auto INTERFACE 
    WITHOUT_NUMPY)
endif()
install(
    TARGETS planning_control_auto
    EXPORT install_targets
)


add_executable(main main.cpp)
target_link_libraries(main PRIVATE planning_control_auto)


add_subdirectory(planning bin)

#设置生成可执行文件的保存路径
add_subdirectory(control bin_control)