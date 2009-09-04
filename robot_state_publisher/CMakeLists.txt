cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

rosbuild_add_library(${PROJECT_NAME} src/joint_state_listener.cpp src/robot_state_publisher.cpp src/treefksolverposfull_recursive.cpp)

rosbuild_add_executable(state_publisher src/state_publisher.cpp )
target_link_libraries(state_publisher ${PROJECT_NAME})

rosbuild_add_executable(test_publisher test/test_publisher.cpp )
target_link_libraries(test_publisher ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_publisher)
rosbuild_declare_test(test_publisher)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_publisher.launch)

# Download needed data file
rosbuild_download_test_data(http://pr.willowgarage.com/data/robot_state_publisher/joint_states.bag test/joint_states.bag)
