cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ivcon)

#uncomment for profiling
set(ROS_LINK_FLAGS "-lm" ${ROS_LINK_FLAGS})

rospack_add_executable(bin/ivcon src/ivcon.c)

