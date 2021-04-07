set(tests
     urdf_unit_test.cpp)

#################################################
# Build all the tests
foreach(test_file ${tests})
  string(REGEX REPLACE ".cpp" "" BINARY_NAME ${test_file})

  add_executable(${BINARY_NAME} ${test_file})
  target_link_libraries(${BINARY_NAME}
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
