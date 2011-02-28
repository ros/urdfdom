set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

rosbuild_add_gtest(test/utest test.cpp)
target_link_libraries(test/utest ${PROJECT_NAME})
