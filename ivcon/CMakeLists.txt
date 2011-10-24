cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

#uncomment for profiling
set(ROS_LINK_FLAGS "-lm" ${ROS_LINK_FLAGS})

rosbuild_add_executable(bin/ivcon src/ivcon.c)
target_link_libraries(bin/ivcon m)

