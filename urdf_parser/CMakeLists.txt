macro(add_urdfdom_library)
  set(oneValueArgs LIBNAME)
  set(multiValueArgs SOURCES LINK)
  cmake_parse_arguments(add_urdfdom_library "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${add_urdfdom_library_LIBNAME} SHARED
    ${add_urdfdom_library_SOURCES})
  add_library(urdfdom::${add_urdfdom_library_LIBNAME} ALIAS ${add_urdfdom_library_LIBNAME})
  target_include_directories(${add_urdfdom_library_LIBNAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
  target_link_libraries(${add_urdfdom_library_LIBNAME} PUBLIC
    ${add_urdfdom_library_LINK}
    ${urdfdom_headers_link_libs}
  )
  target_link_libraries(${add_urdfdom_library_LIBNAME} PRIVATE
    ${console_bridge_link_libs}
    tinyxml2::tinyxml2
  )
  if(NOT CMAKE_CXX_STANDARD)
    target_compile_features(${add_urdfdom_library_LIBNAME} PUBLIC cxx_std_14)
  endif()
  set_target_properties(${add_urdfdom_library_LIBNAME} PROPERTIES
    DEFINE_SYMBOL URDFDOM_EXPORTS
    SOVERSION ${URDF_MAJOR_MINOR_VERSION})
endmacro()

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

add_urdfdom_library(
  LIBNAME
    urdfdom_world
  SOURCES
    src/pose.cpp
    src/model.cpp
    src/link.cpp
    src/joint.cpp
    src/world.cpp)

add_urdfdom_library(
  LIBNAME
    urdfdom_model
  SOURCES
    src/pose.cpp
    src/model.cpp
    src/link.cpp
    src/joint.cpp)

add_urdfdom_library(
  LIBNAME
    urdfdom_sensor
  SOURCES
    src/urdf_sensor.cpp
  LINK
    urdfdom_model)

add_library(urdf_parser INTERFACE)
target_include_directories(urdf_parser INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(urdf_parser INTERFACE
  urdfdom::urdfdom_model
  urdfdom::urdfdom_sensor
  urdfdom::urdfdom_world)

# --------------------------------

add_executable(check_urdf src/check_urdf.cpp)
target_include_directories(check_urdf PUBLIC include)
target_link_libraries(check_urdf urdfdom_model urdfdom_world)

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
  urdf_parser
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
