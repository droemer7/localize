#!/bin/bash

# ------------------------
# Help
# ------------------------
help()
{
  echo "
Options: [-p package] [-b build] [-x execute] [-c]
-p package    Package to build: all, <package name> (default: all)
-b build      Build setting: last, debug, release (default: last)
-m mode       Run mode: build, sim, real, test (default: build)
-c            Enable teleop control
-h            Help
  "
  exit 1
}

# ------------------------
# Arguments
# ------------------------
pkg="all"
build="last"
mode="build"
control="false"

while getopts "p:b:m:ch" opt
do
  case "$opt" in
    p ) pkg="$OPTARG" ;;
    b ) build="$OPTARG" ;;
    m ) mode="$OPTARG" ;;
    c ) control="true" ;;
    h ) help ;;
  esac
done

# ------------------------
# Build Configuration
# ------------------------
# Package name
if [ $pkg == "all" ]
then
  pkg=""
else
  pkg="--pkg $pkg"
fi

# CMake flags
if [ $build == "last" ]
then
  cmake_flags=""
elif [ $build == "debug" ]
then
  cmake_flags="-DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-O0"
elif [ $build == "release" ]
then
  cmake_flags="-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-O3"
else
  echo ""
  echo "$0: Illegal build setting (-b) '$build'"
  echo ""
  exit
fi

# Build
source /opt/ros/noetic/setup.bash
source ~/sw/ros/master/devel/setup.bash
roscd && cd ..
catkin_make $pkg -j4 $cmake_flags

# ------------------------
# Run
# ------------------------
if [ $mode == "real" ]
then
  roslaunch mcl mcl.launch real:=true control:=$control
elif [ $mode == "sim" ]
then
  roslaunch mcl mcl.launch real:=false control:=$control
elif [ $mode == "test" ]
then
  ./devel/lib/mcl/test
elif [ $mode != "build" ]
then
  echo ""
  echo "$0: Illegal mode option (-m) '$mode'"
  help
  exit
fi