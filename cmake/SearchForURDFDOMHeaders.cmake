set (urdfdom_headers_include_dirs "" CACHE STRING "urdfdom_headers include paths. Use this to override automatic detection.")
set (urdfdom_headers_cflags "" CACHE STRING "urdfdom_headers Use this cflag to enable string support.")

SET (urdf_lflags "" CACHE STRING "Linker flags such as rpath for URDF executable.")

include (FindPkgConfig)

########################################
# Find packages
if (PKG_CONFIG_FOUND)

  #################################################
  # Find urdfdom_headers
  pkg_check_modules(urdfdom_headers urdfdom_headers)
  if (NOT urdfdom_headers_FOUND)
    message(STATUS "urdfdom_headers system package not found, using passed in paths")
    ########################################
    # Find urdfdom_headers
    if (NOT urdfdom_headers_include_dirs)

      message(STATUS "urdfdom_headers has no passed in paths, try to auto detect manually.")

      find_path(urdfdom_headers_include_dir urdf_model/link.h ${urdfdom_headers_include_dirs} ENV CPATH)
      
      if (NOT urdfdom_headers_include_dir)
        message (STATUS "Looking for urdf_model/link.h - not found.")
        set (urdfdom_headers_include_dirs /usr/include CACHE STRING
          "urdfdom_headers include paths. Use this to override automatic detection.")
      else (NOT urdfdom_headers_include_dir)
        message (STATUS "Looking for urdf_model/link.h - found")
        set (assim_include_dirs ${urdfdom_headers_include_dir} CACHE STRING
          "urdfdom_headers include paths. Use this to override automatic detection.")
      endif (NOT urdfdom_headers_include_dir)
      
      if (NOT urdfdom_headers_include_dir)
        BUILD_ERROR("Missing: urdfdom_headers")
      endif (NOT urdfdom_headers_include_dir)

    endif (NOT urdfdom_headers_include_dirs)
  else ()
    set(urdfdom_headers_include_dirs ${urdfdom_headers_INCLUDE_DIRS} CACHE STRING "urdfdom_headers include paths. Use this to override automatic detection." FORCE)

    set(urdfdom_headers_cflags ${urdfdom_headers_CFLAGS} CACHE STRING "urdfdom_headers Use this cflag to enable string support." FORCE)
  endif ()

else (PKG_CONFIG_FOUND)
  set (BUILD_URDF OFF CACHE INTERNAL "Build URDF" FORCE)
  BUILD_ERROR ("Error: pkg-config not found")
endif (PKG_CONFIG_FOUND)
