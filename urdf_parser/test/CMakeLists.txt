# urdfdom defines its own uninstall target, so keep ament_cmake_core, which is
# pulled in by ament_cmake_gtest, from defining a conflicting one.
set(AMENT_CMAKE_UNINSTALL_TARGET OFF CACHE BOOL "")

find_package(ament_cmake_gtest REQUIRED)

# unit test to fix geometry problems
set(tests
     urdf_double_convert.cpp
     urdf_schema_capsule_geometry_test.cpp
     urdf_schema_quaternion_test.cpp
     urdf_unit_test.cpp
     urdf_version_test.cpp
)

#################################################
# Build all the tests
foreach(GTEST_SOURCE_file ${tests})
  string(REGEX REPLACE ".cpp" "" BINARY_NAME ${GTEST_SOURCE_file})

  ament_add_gtest_executable(${BINARY_NAME} ${GTEST_SOURCE_file})
  target_link_libraries(${BINARY_NAME} urdfdom_model)

  # Run each test twice, once with the default locale and once with a locale
  # that uses a comma as the decimal separator.
  ament_add_gtest_test(${BINARY_NAME}
    TIMEOUT 240
    ENV LC_ALL=C
  )
  ament_add_gtest_test(${BINARY_NAME}
    TEST_NAME ${BINARY_NAME}_locale
    TIMEOUT 240
    ENV LC_ALL=nl_NL.UTF-8
  )
endforeach()
