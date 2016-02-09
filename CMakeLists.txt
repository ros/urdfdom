cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (urdfdom CXX C)

set (URDF_MAJOR_VERSION 0)
set (URDF_MINOR_VERSION 4)
set (URDF_PATCH_VERSION 0)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})
set (URDF_MAJOR_MINOR_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION})

message (STATUS "${PROJECT_NAME} version ${URDF_VERSION}")

include(GNUInstallDirs)

# hack: by default this would be 'lib/x86_64-linux-gnu'
set(CMAKE_INSTALL_LIBDIR lib)

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

# Using dynamic linking by default
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# If building static libraries, set URDFDOM_STATIC definition for symbol
# visibility settings.
if (BUILD_SHARED_LIBS)
  set(URDFDOM_STATIC FALSE)
else()
  set(URDFDOM_STATIC TRUE)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include_directories("${CMAKE_CURRENT_BINARY_DIR}/urdf_parser/include/urdf_parser")

find_package(TinyXML REQUIRED)
include_directories(SYSTEM ${TinyXML_INCLUDE_DIRS})

find_package(urdfdom_headers 0.4 REQUIRED)
include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS})

find_package(console_bridge 0.3 REQUIRED)
include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
link_directories(${console_bridge_LIBRARY_DIRS})

find_package(Boost REQUIRED system thread unit_test_framework)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

#In Visual Studio a special postfix for 
#libraries compiled in debug is used
if(MSVC)
set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

enable_testing()

add_subdirectory(urdf_parser)

if(WIN32 AND NOT CYGWIN)
  set(CMAKE_CONFIG_INSTALL_DIR CMake)
else()
  set(CMAKE_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake/)
endif()

set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES urdfdom_sensor urdfdom_model_state urdfdom_model urdfdom_world)
set(PKG_DEPENDS urdfdom_headers console_bridge)
set(cmake_conf_file "cmake/urdfdom-config.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in" "${CMAKE_BINARY_DIR}/${cmake_conf_file}" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${cmake_conf_file}
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

set(config_file "urdf_parser/include/urdf_parser/config.h")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${config_file}.in" "${CMAKE_CURRENT_BINARY_DIR}/${config_file}" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${config_file}"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/urdf_parser/ COMPONENT headers)

add_subdirectory(urdf_parser_py)

message(STATUS "Configuration successful. Type make to compile urdfdom")

SET_DIRECTORY_PROPERTIES(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/cmake/urdfdom-config.cmake
  ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/cmake/pkgconfig/urdfdom.pc)
