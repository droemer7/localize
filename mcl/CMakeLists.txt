cmake_minimum_required(VERSION 3.0.2)
project(mcl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  vesc_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mcl
  CATKIN_DEPENDS
    roscpp
    std_msgs
    vesc_msgs
)

include_directories(
  /home/dane/sw/lib/range_libc
  include
  ${catkin_INCLUDE_DIRS}
)

# MCL executable
add_executable(mcl src/main.cpp
                   src/node.cpp
                   src/mcl.cpp
                   src/dist.cpp
                   src/motion.cpp
                   src/sensor.cpp
                   src/util.cpp
)
add_dependencies(mcl
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mcl
  ${catkin_LIBRARIES}
)

# Test executable
add_executable(mcl_test src/test.cpp
                        src/mcl.cpp
                        src/dist.cpp
                        src/motion.cpp
                        src/sensor.cpp
                        src/util.cpp
)
add_dependencies(mcl_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mcl_test
  ${catkin_LIBRARIES}
)