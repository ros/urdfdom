cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_executable(urdf_to_collada src/urdf_to_collada.cpp src/collada_writer.cpp src/stl_loader.cpp)

rosbuild_add_gtest(test_collada_writer test/test_collada_writer.cpp src/collada_writer.cpp src/stl_loader.cpp)
