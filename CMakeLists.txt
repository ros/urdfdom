cmake_minimum_required( VERSION 3.5 FATAL_ERROR )
project (urdfdom CXX C)

set (URDF_MAJOR_VERSION 4)
set (URDF_MINOR_VERSION 0)
set (URDF_PATCH_VERSION 1)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})
set (URDF_MAJOR_MINOR_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION})

message (STATUS "${PROJECT_NAME} version ${URDF_VERSION}")

include(GNUInstallDirs)

option(APPEND_PROJECT_NAME_TO_INCLUDEDIR
  "When ON headers are installed to a folder ending with an extra ${PROJECT_NAME}. \
  This avoids include directory search order issues when overriding this package
  from a merged catkin, ament, or colcon workspace." ON)

if(APPEND_PROJECT_NAME_TO_INCLUDEDIR)
  set(CMAKE_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}")
endif()

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# If compiler support symbol visibility, enable it.
include(CheckCCompilerFlag)
check_c_compiler_flag(-fvisibility=hidden HAS_VISIBILITY)
if (HAS_VISIBILITY)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(tinyxml2_vendor QUIET)
find_package(TinyXML2 REQUIRED)

find_package(urdfdom_headers 1.0 REQUIRED)
find_package(console_bridge_vendor QUIET) # Provides console_bridge 0.4.0 on platforms without it.
find_package(console_bridge REQUIRED)

# Control where libraries and executables are placed during the build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

enable_testing()

set(CMAKE_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake)
string(REGEX REPLACE "[^/]+" ".." RELATIVE_PATH_CMAKE_DIR_TO_PREFIX "${CMAKE_CONFIG_INSTALL_DIR}")
string(REGEX REPLACE "[^/]+" ".." RELATIVE_PATH_LIBDIR_TO_PREFIX "${CMAKE_INSTALL_LIBDIR}")

add_subdirectory(urdf_parser)

set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES urdfdom_sensor urdfdom_model_state urdfdom_model urdfdom_world)
set(PKG_DEPENDS urdfdom_headers)
set(PKG_EXPORTS urdfdom)
set(cmake_conf_file "cmake/urdfdom-config")
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}Version.cmake"
  VERSION ${URDF_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file("${cmake_conf_file}.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}.cmake"
  INSTALL_DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}Version.cmake"
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Make the package config file
set(PKG_DESC "Unified Robot Description Format")
set(PKG_DEPENDS "urdfdom_headers") # make the list separated by spaces instead of ;
set(PKG_URDF_LIBS "-lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world")
set(pkg_conf_file "cmake/pkgconfig/urdfdom.pc")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${pkg_conf_file}.in" "${CMAKE_BINARY_DIR}/${pkg_conf_file}" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${pkg_conf_file}
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)


# Add uninstall target
# Ref: http://www.cmake.org/Wiki/CMake_FAQ#Can_I_do_.22make_uninstall.22_with_CMake.3F
configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/uninstall.cmake.in"
  "${PROJECT_BINARY_DIR}/uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall.cmake")

message(STATUS "Configuration successful. Type make to compile urdfdom")

SET_DIRECTORY_PROPERTIES(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/cmake/urdfdom-config.cmake
  ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/cmake/pkgconfig/urdfdom.pc)
