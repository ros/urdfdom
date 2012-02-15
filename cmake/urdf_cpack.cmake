################################################################################
#Find available package generators

# DEB
if ("${CMAKE_SYSTEM}" MATCHES "Linux")
  find_program(DPKG_PROGRAM dpkg)
  if (EXISTS ${DPKG_PROGRAM})
    list (APPEND CPACK_GENERATOR "DEB")
  endif(EXISTS ${DPKG_PROGRAM})
endif()

list (APPEND CPACK_SOURCE_GENERATOR "TBZ2")
list (APPEND CPACK_SOURCE_GENERATOR "ZIP")
list (APPEND CPACK_SOURCE_IGNORE_FILES ";TODO;/.hg/;.swp$;/build/")

include (InstallRequiredSystemLibraries)

#execute_process(COMMAND dpkg --print-architecture _NPROCE)
set (DEBIAN_PACKAGE_DEPENDS "libtinyxml, libboost-all-dev")

set (URDF_CPACK_CFG_FILE "${PROJECT_BINARY_DIR}/cpack_options.cmake")

################################################################################
# Make the CPack input file
#macro(URDF_MAKE_CPACK_INPUT)
#  set(_cpack_cfg_in "${urdf_cmake_dir}/cpack_options.cmake.in")
#
#  #Prepare the components list
#  #URDF_CPACK_MAKE_COMPS_OPTS(URDF_CPACK_COMPONENTS "${_comps}")
#
#  configure_file(${_cpack_cfg_in} ${URDF_CPACK_CFG_FILE} @ONLY)
#endmacro(URDF_MAKE_CPACK_INPUT)


#macro(URDF_CPACK_MAKE_COMPS_OPTS _var _current)
#    set(_comps_list)
#    foreach(_ss ${URDF_SUBSYSTEMS})
#        URDF_GET_SUBSYS_STATUS(_status ${_ss})
#        if(_status)
#            set(_comps_list "${_comps_list} ${_ss}")
#            URDF_CPACK_ADD_COMP_INFO(${_var} ${_ss})
#        endif(_status)
#    endforeach(_ss)
#    set(${_var} "${${_var}}\nset(CPACK_COMPONENTS_ALL${_comps_list})\n")
#endmacro(URDF_CPACK_MAKE_COMPS_OPTS)

