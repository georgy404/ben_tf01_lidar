cmake_minimum_required(VERSION 2.8.3)
project(ben_tf01_lidar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ben_tf01_lidar
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs
    # DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

# Serial port worker
set(SERIAL_PORT_WORKER
    include/serial_port_worker/serial_port_worker.h
    include/serial_port_worker/serial_port_worker.c
    )

# Ben-TF01 lidar class
set(BEN_TF01
    include/ben_tf01_lidar/tf01_broadcast.h
    src/tf01_broadcast.cpp
    src/main.cpp
    )

## Declare a C++ library
add_library(${PROJECT_NAME}
    ${BEN_TF01}
    ${SERIAL_PORT_WORKER}
    )

add_executable(${PROJECT_NAME}_node 
    ${BEN_TF01}
    ${SERIAL_PORT_WORKER}
    )

add_dependencies(${PROJECT_NAME}_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ben_tf01_lidar
    )
