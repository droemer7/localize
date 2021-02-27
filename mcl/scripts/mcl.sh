#!/bin/bash

# Arguments
build="debug"
run="build"
standalone="0"

while getopts "b:r:s:" opt
do
  case "$opt" in
    b ) build="$OPTARG" ;;
    r ) run="$OPTARG" ;;
    s ) standalone="$OPTARG" ;;
  esac
done

# Change to catkin workspace
source /opt/ros/noetic/setup.bash
source ~/sw/ros/master/devel/setup.bash
roscd && cd ..

# Build
if [ $build == "debug" ]
then
  catkin_make --pkg mcl -j4 -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-O0
else
  catkin_make --pkg mcl -j4 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-O3
fi

# Run
if [ $run == "real" ] || [ $run == "sim" ] || [ $run == "test" ]
then
  if [ $run == "real" ]
  then
    roslaunch mcl mcl.launch real:=true standalone:=$standalone
  else
    roslaunch mcl mcl.launch real:=false standalone:=$standalone
  fi

  if [ $run == "test" ]
  then
    ./devel/lib/mcl/mcl_test
  fi
fi