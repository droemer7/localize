cmake_minimum_required(VERSION 3.0.2)
project(amcl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  range_libc
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  vesc_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    range_libc
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    vesc_msgs
)

include_directories(
  include
  ${range_libc_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(amcl
  src/main.cpp
  src/node.cpp
  src/amcl.cpp
  src/distribution.cpp
  src/histogram.cpp
  src/motion.cpp
  src/sensor.cpp
  src/common.cpp
)

target_link_libraries(amcl ${catkin_LIBRARIES})