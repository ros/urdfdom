add_library(urdfdom_world SHARED
  src/pose.cpp
  src/model.cpp
  src/link.cpp
  src/joint.cpp
  src/world.cpp)
target_include_directories(urdfdom_world SYSTEM PUBLIC
  ${TinyXML_INCLUDE_DIRS})

if(TARGET console_bridge::console_bridge)
  set(console_bridge_link_libs "console_bridge::console_bridge")
else()
  set(console_bridge_link_libs "console_bridge")
endif()

if(TARGET urdfdom_headers::urdfdom_headers)
  set(urdfdom_headers_link_libs "urdfdom_headers::urdfdom_headers")
else()
  set(urdfdom_headers_link_libs "")
endif()

target_include_directories(urdfdom_world PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(urdfdom_world PUBLIC
  ${console_bridge_link_libs}
  ${urdfdom_headers_link_libs}
  ${TinyXML_LIBRARIES})
set_target_properties(urdfdom_world PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_model SHARED
  src/pose.cpp
  src/model.cpp
  src/link.cpp
  src/joint.cpp)
target_include_directories(urdfdom_model SYSTEM PUBLIC
  ${TinyXML_INCLUDE_DIRS})
target_include_directories(urdfdom_model PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(urdfdom_model PUBLIC
  ${console_bridge_link_libs}
  ${urdfdom_headers_link_libs}
  ${TinyXML_LIBRARIES})
set_target_properties(urdfdom_model PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_sensor SHARED src/urdf_sensor.cpp)
target_include_directories(urdfdom_sensor SYSTEM PUBLIC
  ${TinyXML_INCLUDE_DIRS})
target_include_directories(urdfdom_sensor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(urdfdom_sensor PUBLIC
  urdfdom_model
  ${console_bridge_link_libs}
  ${urdfdom_headers_link_libs}
  ${TinyXML_LIBRARIES})
set_target_properties(urdfdom_sensor PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_model_state SHARED
  src/urdf_model_state.cpp
  src/twist.cpp)
target_include_directories(urdfdom_model_state SYSTEM PUBLIC
  ${TinyXML_INCLUDE_DIRS})
target_include_directories(urdfdom_model_state PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(urdfdom_model_state
  ${console_bridge_link_libs}
  ${urdfdom_headers_link_libs}
  ${TinyXML_LIBRARIES})
set_target_properties(urdfdom_model_state PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

# --------------------------------

add_executable(check_urdf src/check_urdf.cpp)
target_include_directories(check_urdf PUBLIC include)
target_link_libraries(check_urdf urdfdom_model urdfdom_world)

# Deprecated executable
add_executable(urdf_to_graphiz src/urdf_to_graphviz.cpp)
target_link_libraries(urdf_to_graphiz urdfdom_model)

add_executable(urdf_to_graphviz src/urdf_to_graphviz.cpp)
target_include_directories(urdf_to_graphviz PUBLIC include)
target_link_libraries(urdf_to_graphviz urdfdom_model)

# urdf_mem_test is a binary for testing, not a unit test
add_executable(urdf_mem_test test/memtest.cpp)
target_include_directories(urdf_mem_test PUBLIC include)
target_link_libraries(urdf_mem_test urdfdom_model)

include(CTest)
if(BUILD_TESTING)
  # TODO: check Shane's comment https://github.com/ros/urdfdom/pull/157/files#r664960227
  add_subdirectory(test)
endif()

INSTALL(
  TARGETS
  check_urdf
  urdf_to_graphviz
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
