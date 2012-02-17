cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)
enable_testing()

project (urdf)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set (URDF_MAJOR_VERSION 0)
set (URDF_MINOR_VERSION 1)
set (URDF_PATCH_VERSION 0)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${URDF_VERSION}")

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set (CMAKE_INSTALL_PREFIX "/usr/" CACHE PATH "URDF install prefix" FORCE)
endif()

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set (USE_FULL_RPATH OFF CACHE BOOL "Set to true to enable full rpath")

if (USE_FULL_RPATH)
  # use, i.e. don't skip the full RPATH for the build tree
  set(CMAKE_SKIP_BUILD_RPATH  FALSE)
  
  # when building, don't use the install RPATH already
  # (but later on when installing)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  
  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  
  # the RPATH to be used when installing, but only if its not a system directory
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  endif("${isSystemDir}" STREQUAL "-1")
endif()

set (BUILD_URDF ON CACHE INTERNAL "Build URDF" FORCE)

set (INCLUDE_INSTALL_DIR "include")
set (LIB_INSTALL_DIR "lib")
set (BIN_INSTALL_DIR "bin")


# with -fPIC
if(UNIX AND NOT WIN32)
 find_program(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
 if(CMAKE_UNAME)
   exec_program(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
   set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL
     "processor type (i386 and x86_64)")
   if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
     ADD_DEFINITIONS(-fPIC)
   endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
 endif(CMAKE_UNAME)
endif(UNIX AND NOT WIN32)

set (MIN_BOOST_VERSION 1.40.0 CACHE INTERNAL "Boost min version requirement" FORCE)

set (urdf_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH 
     "Location of CMake scripts")

include (${urdf_cmake_dir}/DissectVersion.cmake)

message (STATUS "\n\n====== Finding 3rd Party Packages ======")
include (${urdf_cmake_dir}/SearchForStuff.cmake)
message (STATUS "----------------------------------------\n")

#####################################
# Build type cflags
set (CMAKE_C_FLAGS_ALL " -Wall -Wno-long-long -Wno-unused-value -Wno-unused-value -Wno-unused-value -Wno-unused-value -fPIC -pedantic" CACHE INTERNAL "General C Flags")

set (CMAKE_C_FLAGS_RELEASE " -s -O2 -DNDEBUG ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release" FORCE)
set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})

set (CMAKE_C_FLAGS_DEBUG " -ggdb3 -Wextra ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for debug" FORCE)
set (CMAKE_CXX_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})

#####################################
# Build type link flags
set (CMAKE_LINK_FLAGS_RELEASE " -fPIC" CACHE INTERNAL "Link flags fro release" FORCE)
set (CMAKE_LINK_FLAGS_DEBUG " -fPIC" CACHE INTERNAL "Link flags for debug" FORCE)

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "release" CACHE STRING 
    "Choose the type of build, options are: debug release" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)

#####################################
# Set all the global build flags
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")

if ( "${CMAKE_BUILD_TYPE}" STREQUAL "RELEASE")
  set (BUILD_TYPE_RELEASE TRUE)
else()
  set (BUILD_TYPE_RELEASE FALSE)
endif()

if ( "${CMAKE_BUILD_TYPE}" STREQUAL "DEBUG")
  set (BUILD_TYPE_DEBUG TRUE)
else()
  set (BUILD_TYPE_DEBUG FALSE)
endif()

if ( build_warnings )
  message(STATUS "BUILD WARNINGS")
  foreach (msg ${build_warnings})
    message(STATUS ${msg})
  endforeach (msg ${build_warnings})
  message(STATUS "END BUILD WARNINGS\n")
endif (build_warnings)


if ( build_errors )
  message(STATUS "BUILD ERRORS: These must be resolved before compiling.")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach (msg ${build_errors})
  message(STATUS "END BUILD ERRORS\n")
  message (FATAL_ERROR "Errors encountered in build.")
else (build_errors)

  include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
  
  message (STATUS "C Flags:${CMAKE_CXX_FLAGS}")
  message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
  message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")
 
  if (BUILD_URDF)
    add_subdirectory(urdf_interface)
    add_subdirectory(urdf_parser)
  endif (BUILD_URDF)

  ########################################
  # Make the package config files
  set (pkgconfig_files urdf_interface urdf_parser)

  foreach (pkgconfig ${pkgconfig_files})
    configure_file(${CMAKE_SOURCE_DIR}/cmake/pkgconfig/${pkgconfig}.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/pkgconfig/${pkgconfig}.pc @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/pkgconfig/${pkgconfig}.pc DESTINATION lib/pkgconfig COMPONENT pkgconfig)
  endforeach()

  ########################################
  # Package Creation: 
  include (${urdf_cmake_dir}/urdf_cpack.cmake)
  set (CPACK_PACKAGE_VERSION "${URDF_VERSION}")
  set (CPACK_PACKAGE_VERSION_MAJOR "${URDF_MAJOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_MINOR "${URDF_MINOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_PATCH "${URDF_PATCH_VERSION}")

  if (CPACK_GENERATOR)
    message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")

    configure_file("${urdf_cmake_dir}/cpack_options.cmake.in" ${URDF_CPACK_CFG_FILE} @ONLY)
    set(CPACK_PROJECT_CONFIG_FILE ${URDF_CPACK_CFG_FILE})
    include (CPack)
  endif()

  message(STATUS "Configuration successful. Type make to compile urdf")
endif(build_errors)

