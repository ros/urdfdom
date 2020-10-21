cmake_minimum_required( VERSION 3.5 FATAL_ERROR )
project (urdfdom VERSION 2.3.3 LANGUAGES CXX C)

set (URDF_MAJOR_VERSION 1)
set (URDF_MINOR_VERSION 0)
set (URDF_PATCH_VERSION 0)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})
set (URDF_MAJOR_MINOR_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION})

message (STATUS "${PROJECT_NAME} version ${URDF_VERSION}")

include(GNUInstallDirs)

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

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

find_package(urdfdom_headers 1.0 REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(console_bridge_vendor REQUIRED) # Provides console_bridge 0.4.0 on platforms without it.
find_package(console_bridge REQUIRED)

#In Visual Studio a special postfix for 
#libraries compiled in debug is used
if(MSVC)
set(CMAKE_DEBUG_POSTFIX "d")
# Export symbols on Windows by default
add_definitions(-DURDFDOM_EXPORTS)
endif(MSVC)

# Control where libraries and executables are placed during the build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

enable_testing()

if(WIN32 AND NOT CYGWIN)
  set(CMAKE_CONFIG_INSTALL_DIR CMake)
else()
  set(CMAKE_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake)
endif()
string(REGEX REPLACE "[^/]+" ".." RELATIVE_PATH_CMAKE_DIR_TO_PREFIX "${CMAKE_CONFIG_INSTALL_DIR}")
string(REGEX REPLACE "[^/]+" ".." RELATIVE_PATH_LIBDIR_TO_PREFIX "${CMAKE_INSTALL_LIBDIR}")

add_subdirectory(urdf_parser)

set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES urdfdom_sensor urdfdom_model_state urdfdom_model urdfdom_world)
set(PKG_DEPENDS urdfdom_headers console_bridge)
set(PKG_EXPORTS urdfdom)
set(cmake_conf_file "cmake/urdfdom-config.cmake")
set(cmake_ver_file "${CMAKE_BINARY_DIR}/cmake/urdfdom-config-version.cmake")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${cmake_ver_file}" VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion
)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in" "${CMAKE_BINARY_DIR}/${cmake_conf_file}" @ONLY)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_BINARY_DIR}/${cmake_conf_file} ${cmake_ver_file}
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR} COMPONENT cmake)

# Make the package config file
if (NOT MSVC)
  set(PKG_DESC "Unified Robot Description Format")
  set(PKG_DEPENDS "urdfdom_headers console_bridge") # make the list separated by spaces instead of ;
  set(PKG_URDF_LIBS "-lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world")
  set(pkg_conf_file "cmake/pkgconfig/urdfdom.pc")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${pkg_conf_file}.in" "${CMAKE_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES ${CMAKE_BINARY_DIR}/${pkg_conf_file}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
endif()

message(STATUS "Configuration successful. Type make to compile urdfdom")

SET_DIRECTORY_PROPERTIES(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/cmake/urdfdom-config.cmake
  ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/cmake/pkgconfig/urdfdom.pc)
