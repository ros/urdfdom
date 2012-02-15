include (${urdf_cmake_dir}/URDFUtils.cmake)
include (CheckCXXSourceCompiles)

set (tinyxml_include_dirs "" CACHE STRING "Tinyxml include paths. Use this to override automatic detection.")
set (tinyxml_library_dirs "" CACHE STRING "Tinyxml library paths. Use this to override automatic detection.")
set (tinyxml_libraries "" CACHE STRING "Tinyxml libraries Use this to override automatic detection.")
set (tinyxml_cflags "" CACHE STRING "Tinyxml Use this cflag to enable string support.")

set (boost_include_dirs "" CACHE STRING "Boost include paths. Use this to override automatic detection.")
set (boost_library_dirs "" CACHE STRING "Boost library paths. Use this to override automatic detection.")
set (boost_libraries "" CACHE STRING "Boost libraries. Use this to override automatic detection.")

SET (urdf_lflags "" CACHE STRING "Linker flags such as rpath for URDF executable.")

include (${urdf_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Find packages
if (PKG_CONFIG_FOUND)

  #################################################
  # Find tinyxml
  pkg_check_modules(tinyxml tinyxml)
  if (NOT tinyxml_FOUND)
    message(STATUS "tinyxml system package not found, using passed in paths")
    ########################################
    # Find tinyxml
    if (NOT tinyxml_include_dirs AND NOT tinyxml_library_dirs AND NOT tinyxml_libraries )

      message(STATUS "tinyxml has no passed in paths, try to auto detect manually.")

      find_path(tinyxml_include_dir tinyxml/tinyxml.hpp ${tinyxml_include_dirs} ENV CPATH)
      
      if (NOT tinyxml_include_dir)
        message (STATUS "Looking for tinyxml/tinyxml.hpp - not found.")
        set (tinyxml_include_dirs /usr/include CACHE STRING
          "tinyxml include paths. Use this to override automatic detection.")
      else (NOT tinyxml_include_dir)
        message (STATUS "Looking for tinyxml/tinyxml.hpp - found")
        set (assim_include_dirs ${tinyxml_include_dir} CACHE STRING
          "tinyxml include paths. Use this to override automatic detection.")
      endif (NOT tinyxml_include_dir)
      
      find_library(tinyxml_library tinyxml ENV LD_LIBRARY_PATH)
      
      if (tinyxml_library)
        message (STATUS "Looking for libtinyxml - found")
        APPEND_TO_CACHED_LIST(tinyxml_libraries
                              "tinyxml libraries Use this to override automatic detection."
                              ${tinyxml_library})
      endif (tinyxml_library)
     
      if (NOT tinyxml_include_dir OR NOT tinyxml_library)
        BUILD_ERROR("Missing: tinyxml")
      endif (NOT tinyxml_include_dir OR NOT tinyxml_library)

    endif (NOT tinyxml_include_dirs AND NOT tinyxml_library_dirs AND NOT tinyxml_libraries )
  else ()
    set(tinyxml_include_dirs ${tinyxml_INCLUDE_DIRS} CACHE STRING "Tinyxml include paths. Use this to override automatic detection." FORCE)

    set(tinyxml_library_dirs ${tinyxml_LIBRARY_DIRS} CACHE STRING "Tinyxml library paths. Use this to override automatic detection." FORCE)

    set(tinyxml_libraries ${tinyxml_LIBRARIES} CACHE STRING "Tinyxml libraries Use this to override automatic detection." FORCE)

    set(tinyxml_cflags ${tinyxml_CFLAGS} CACHE STRING "Tinyxml Use this cflag to enable string support." FORCE)
  endif ()

else (PKG_CONFIG_FOUND)
  set (BUILD_URDF OFF CACHE INTERNAL "Build URDF" FORCE)
  BUILD_ERROR ("Error: pkg-config not found")
endif (PKG_CONFIG_FOUND)

find_package(GTest)
if (GTEST_FOUND)
  enable_testing()
else()
  message (STATUS "  Tests will not be built")
endif()

########################################
# Find Boost, if not specified manually
if (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )

  # Clear some variables to ensure that the checks for boost are 
  # always run
  set (Boost_THREAD_FOUND OFF CACHE INTERNAL "" FORCE)
  set (Boost_SIGNALS_FOUND OFF CACHE INTERNAL "" FORCE)

  set(Boost_ADDITIONAL_VERSIONS "1.35" "1.35.0" "1.36" "1.36.1" "1.37.0" "1.39.0" CACHE INTERNAL "Boost Additional versions" FORCE)

  include (FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED thread signals system filesystem)

  if (NOT Boost_FOUND)
    set (BUILD_URDF OFF CACHE INTERNAL "Build URDF" FORCE)
    BUILD_ERROR ("Boost thread and signals not found. Please install Boost threads and signals version ${MIN_BOOST_VERSION} or higher.")
  endif (NOT Boost_FOUND)

  set (boost_include_dirs ${Boost_INCLUDE_DIRS} CACHE STRING 
    "Boost include paths. Use this to override automatic detection." FORCE)

  set (boost_library_dirs ${Boost_LIBRARY_DIRS} CACHE STRING
    "Boost link dirs. Use this to override automatic detection." FORCE)

  LIST_TO_STRING(tmp "${Boost_LIBRARIES}")
  set (boost_libraries ${tmp} CACHE STRING 
    "Boost libraries. Use this to override automatic detection." FORCE )

endif (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries ) 

set (Boost_DIR "" CACHE INTERNAL "" FORCE)

STRING(REGEX REPLACE "(^| )-L" " " boost_library_dirs "${boost_library_dirs}")
STRING(REGEX REPLACE "(^| )-l" " " boost_libraries "${boost_libraries}")
#STRING(STRIP ${boost_library_dirs} boost_library_dirs)
#STRING(STRIP ${boost_libraries} boost_libraries)
STRING(REGEX REPLACE " " ";" boost_libraries "${boost_libraries}")

