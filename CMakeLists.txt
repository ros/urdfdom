cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (urdfdom CXX C)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set (URDF_MAJOR_VERSION 0)
set (URDF_MINOR_VERSION 1)
set (URDF_PATCH_VERSION 6)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${URDF_VERSION}")

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(SearchForTinyXML)
include(SearchForURDFDOMHeaders)

add_subdirectory(urdf_parser)

# Make the package config file
if (NOT MSVC)
  set(PKG_NAME "urdfdom")
  set(PKG_DESC "Unified Robot Description Format")
  set(PKG_URDF_LIBS "-lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model")
  set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkgconfig/urdfdom.pc")
  configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
  install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
endif()


message(STATUS "Configuration successful. Type make to compile urdfdom")
