include_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}/gtest/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gtest
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

# unit test to fix geometry problems
set(tests
     urdf_unit_test.cpp)

#################################################
# Build all the tests
foreach(GTEST_SOURCE_file ${tests})
  string(REGEX REPLACE ".cpp" "" BINARY_NAME ${GTEST_SOURCE_file})
  add_executable(${BINARY_NAME} ${GTEST_SOURCE_file})

  target_link_libraries(${BINARY_NAME}
    gtest_main
    gtest
    urdfdom_model
  )
  if (UNIX)
    target_link_libraries(${BINARY_NAME} pthread)
  endif()

  add_test(NAME    ${BINARY_NAME}
           COMMAND ${BINARY_NAME}
                   --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

  set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)
endforeach()
