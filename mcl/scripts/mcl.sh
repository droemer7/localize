#!/bin/bash

# Help
help()
{
  echo "
Options: [-b build] [-x execute] [-c]
-b build      Build setting: last, debug, release (default: last, uses cached CMake arguments)
-x execute    Task to execute: build, sim, real, test (default: build)
-c            Run with teleop control
-h            Help
  "
  exit 1
}

# Arguments
build="last"
exec="build"
control="false"

while getopts "b:x:ch" opt
do
  case "$opt" in
    b ) build="$OPTARG" ;;
    x ) exec="$OPTARG" ;;
    c ) control="true" ;;
    h ) help ;;
  esac
done

# Source ROS
source /opt/ros/noetic/setup.bash
source ~/sw/ros/master/devel/setup.bash

# Build
roscd && cd ..
if [ $build == "last" ]
then
  catkin_make --pkg mcl -j4
elif [ $build == "debug" ]
then
  catkin_make --pkg mcl -j4 -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-O0
elif [ $build == "release" ]
then
  catkin_make --pkg mcl -j4 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-O3
else
  echo ""
  echo "$0: Illegal build setting (-b) '$build'"
  echo ""
  exit
fi

# Task
if [ $exec == "real" ]
then
  roslaunch mcl mcl.launch real:=true control:=$control
elif [ $exec == "sim" ]
then
  roslaunch mcl mcl.launch real:=false control:=$control
elif [ $exec == "test" ]
then
  ./devel/lib/mcl/test
elif [ $exec != "build" ]
then
  echo ""
  echo "$0: Illegal execute option (-x) '$exec'"
  help
  exit
fi