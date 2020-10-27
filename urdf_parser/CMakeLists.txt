#[[
It is necessary to add the `SYSTEM` keyword to use a vendored package when
another version of that package exists in a directory that has been included
without the `SYSTEM` keyword by another package.

Specifically, say that you have two packages, `A` and `B`. Both are installed
in `/usr/local`.  Further, assume `A` is vendored in ROS 2, but `B` is not. And
then assume the `CMakeLists.txt` does `find_package(B); find_package(A)`. In
that case, when `B` is found, it will add `-I/usr/local/include` to the compile
flags. And when A is then found, it will add
`-isystem /path/to/install/A/include` to the compile flags. But since `-I`
takes precedence over `-isystem`, we will always use `/usr/local/include/A.hpp`,
even though we really want to use the vendored header files. In this case, by
making all include files `SYSTEM`, we make them all be `-isystem`, and then the
compiler respects the command-line ordering.

In this particular case, requiring TinyXML in this package added
`-I/usr/local/include` to the build command and caused Clang to select the
system's `console_bridge`, instead of the vendored version. To select the
`console_bridge` in `console_bridge_vendor`, we need to use the `SYSTEM` keyword
when including TinyXML for the compiler to respect the command-line ordering and
to select the vendored version of `console_bridge`.
]]
add_library(urdfdom_world SHARED
  src/pose.cpp
  src/model.cpp
  src/link.cpp
  src/joint.cpp
  src/world.cpp)
target_include_directories(urdfdom_world SYSTEM PUBLIC
  ${TinyXML_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(urdfdom_world PUBLIC
  console_bridge::console_bridge
  urdfdom_headers::urdfdom_headers
  ${TinyXML_LIBRARIES})
set_target_properties(urdfdom_world PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_model SHARED
  src/pose.cpp
  src/model.cpp
  src/link.cpp
  src/joint.cpp)
target_include_directories(urdfdom_model SYSTEM PUBLIC
  ${TinyXML_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(urdfdom_model PUBLIC
  console_bridge::console_bridge
  urdfdom_headers::urdfdom_headers
  ${TinyXML_LIBRARIES})
set_target_properties(urdfdom_model PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_sensor SHARED src/urdf_sensor.cpp)
target_include_directories(urdfdom_sensor SYSTEM PUBLIC
  ${TinyXML_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(urdfdom_sensor PUBLIC
  urdfdom_model
  console_bridge::console_bridge
  urdfdom_headers::urdfdom_headers
  ${TinyXML_LIBRARIES})
set_target_properties(urdfdom_sensor PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_model_state SHARED
  src/urdf_model_state.cpp
  src/twist.cpp)
target_include_directories(urdfdom_model_state SYSTEM PUBLIC
  ${TinyXML_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(urdfdom_model_state
  console_bridge::console_bridge
  urdfdom_headers::urdfdom_headers
  ${TinyXML_LIBRARIES})
set_target_properties(urdfdom_model_state PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

# --------------------------------

add_executable(check_urdf src/check_urdf.cpp)
target_include_directories(check_urdf PUBLIC include)
target_link_libraries(check_urdf urdfdom_model urdfdom_world)

add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
target_include_directories(urdf_to_graphiz PUBLIC include)
target_link_libraries(urdf_to_graphiz urdfdom_model)

# urdf_mem_test is a binary for testing, not a unit test
add_executable(urdf_mem_test test/memtest.cpp)
target_include_directories(urdf_mem_test PUBLIC include)
target_link_libraries(urdf_mem_test urdfdom_model)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

INSTALL(
  TARGETS
  check_urdf
  urdf_to_graphiz
  urdf_mem_test
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
INSTALL(
  TARGETS
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  urdfdom_model_state
  EXPORT
  urdfdom
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(
  EXPORT urdfdom
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
  NAMESPACE "${PROJECT_NAME}::"
  FILE "urdfdomExport.cmake"
)

INSTALL(DIRECTORY include/urdf_parser DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
