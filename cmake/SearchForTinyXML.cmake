set (tinyxml_include_dirs "" CACHE STRING "Tinyxml include paths. Use this to override automatic detection.")
set (tinyxml_library_dirs "" CACHE STRING "Tinyxml library paths. Use this to override automatic detection.")
set (tinyxml_libraries "" CACHE STRING "Tinyxml libraries Use this to override automatic detection.")
set (tinyxml_cflags "" CACHE STRING "Tinyxml Use this cflag to enable string support.")

SET (urdf_lflags "" CACHE STRING "Linker flags such as rpath for URDF executable.")

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
