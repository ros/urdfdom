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

rosbuild_add_library(${PROJECT_NAME} src/xml_parser.cpp src/dom_parser.cpp)

rosbuild_add_executable(example_xml test/example_xml.cpp  )
target_link_libraries(example_xml ${PROJECT_NAME})

rosbuild_add_executable(example_dom test/example_dom.cpp )
target_link_libraries(example_dom ${PROJECT_NAME})

rosbuild_add_executable(test_parser test/test_kdl_parser.cpp )
target_link_libraries(test_parser ${PROJECT_NAME})

rosbuild_add_gtest_build_flags(test_parser)
rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_kdl_parser.launch)
