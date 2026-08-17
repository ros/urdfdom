include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/gtest/include
  ${CMAKE_CURRENT_SOURCE_DIR}/gtest
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
target_compile_features(gtest PUBLIC cxx_std_11)

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)

# unit test to fix geometry problems
set(tests
     urdf_double_convert.cpp
     urdf_schema_capsule_geometry_test.cpp
     urdf_schema_full_robot_test.cpp
     urdf_schema_geometry_test.cpp
     urdf_schema_joint_axis_test.cpp
     urdf_schema_joint_calibration_test.cpp
     urdf_schema_joint_dynamics_test.cpp
     urdf_schema_joint_limits_test.cpp
     urdf_schema_joint_mimic_test.cpp
     urdf_schema_joint_safety_test.cpp
     urdf_schema_joint_types_test.cpp
     urdf_schema_link_inertial_test.cpp
     urdf_schema_material_test.cpp
     urdf_schema_model_test.cpp
     urdf_schema_origin_test.cpp
     urdf_schema_quaternion_test.cpp
     urdf_schema_topology_test.cpp
     urdf_schema_visual_collision_test.cpp
     urdf_unit_test.cpp
     urdf_version_test.cpp
)

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

  target_compile_definitions(${BINARY_NAME} PRIVATE
    TEST_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")

  add_test(NAME    ${BINARY_NAME}
           COMMAND ${BINARY_NAME}
                   --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

  set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240 ENVIRONMENT LC_ALL=C)

  add_test(NAME    ${BINARY_NAME}_locale
           COMMAND ${BINARY_NAME}
                   --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}_locale.xml)

  set_tests_properties(${BINARY_NAME}_locale PROPERTIES TIMEOUT 240 ENVIRONMENT LC_ALL=nl_NL.UTF-8)
endforeach()
