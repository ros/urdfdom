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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(${PROJECT_NAME} src/robot_state_publisher.cpp src/treefksolverposfull_recursive.cpp)

rosbuild_add_executable(state_publisher src/joint_state_listener.cpp )
target_link_libraries(state_publisher ${PROJECT_NAME})

rosbuild_add_executable(test_publisher test/test_publisher.cpp )
target_link_libraries(test_publisher ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_publisher)
rosbuild_declare_test(test_publisher)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_publisher.launch)

rosbuild_add_executable(test_one_link test/test_one_link.cpp )
target_link_libraries(test_one_link ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_one_link)
rosbuild_declare_test(test_one_link)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_one_link.launch)

rosbuild_add_executable(test_two_links_fixed_joint test/test_two_links_fixed_joint.cpp )
target_link_libraries(test_two_links_fixed_joint ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_two_links_fixed_joint)
rosbuild_declare_test(test_two_links_fixed_joint)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_fixed_joint.launch)

rosbuild_add_executable(test_two_links_moving_joint test/test_two_links_moving_joint.cpp )
target_link_libraries(test_two_links_moving_joint ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_two_links_moving_joint)
rosbuild_declare_test(test_two_links_moving_joint)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_moving_joint.launch)

# Download needed data file
rosbuild_download_test_data(http://pr.willowgarage.com/data/robot_state_publisher/joint_states_indexed.bag test/joint_states_indexed.bag 793e0b566ebe4698265a936b92fa2bba)
