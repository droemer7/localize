cmake_minimum_required(VERSION 3.0.2)
project(amcl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  std_msgs
  vesc_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES amcl
  CATKIN_DEPENDS
    roscpp
    tf2_ros
    std_msgs
    vesc_msgs
)

include_directories(
  /home/dane/sw/lib/range_libc
  include
  ${catkin_INCLUDE_DIRS}
)

# AMCL executable
add_executable(amcl src/main.cpp
                    src/node.cpp
                    src/amcl.cpp
                    src/distribution.cpp
                    src/histogram.cpp
                    src/motion.cpp
                    src/sensor.cpp
                    src/common.cpp
)
add_dependencies(amcl
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(amcl
  ${catkin_LIBRARIES}
)