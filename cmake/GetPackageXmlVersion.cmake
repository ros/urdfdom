# Copyright (C) 2024 Open Source Robotics Foundation
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copied from https://github.com/gazebosim/gz-cmake/blob/gz-cmake5_5.0.0/cmake/GzGetPackageXmlVersion.cmake
# and renamed.

#################################################
# get_package_xml_version(<package_xml_path> <version_var_prefix>)
#
# Given the path to a package.xml file in <package_xml_path>,
# extract the version number and return the following variables
# prefixed by <version_var_prefix>:
# - <version_var_prefix>_VERSION: the full version number (Major.Minor.Patch)
# - <version_var_prefix>_VERSION_MAJOR: the major version number
# - <version_var_prefix>_VERSION_MINOR: the minor version number
# - <version_var_prefix>_VERSION_PATCH: the patch version number
function(get_package_xml_version package_xml_path version_var_prefix)

  if(NOT Python3_Interpreter_FOUND)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
  endif()
  execute_process(
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/print_package_xml_version.py"
            "${package_xml_path}"
    OUTPUT_VARIABLE PACKAGE_XML_version
    ERROR_VARIABLE  PACKAGE_XML_error
    RESULT_VARIABLE PACKAGE_XML_result)
  if(NOT ${PACKAGE_XML_result} EQUAL 0)
    message("")
    message(FATAL_ERROR "Failed to parse version number from package.xml: ${PACKAGE_XML_error}")
  endif()
  # split version number into list of three numbers
  string(REPLACE "." ";" PACKAGE_XML_version_list ${PACKAGE_XML_version})

  # Create variables for major, minor, and patch version numbers
  list(GET PACKAGE_XML_version_list 0 PACKAGE_XML_version_major)
  list(GET PACKAGE_XML_version_list 1 PACKAGE_XML_version_minor)
  list(GET PACKAGE_XML_version_list 2 PACKAGE_XML_version_patch)

  # Return variables for the full version number as well as major, minor, and patch version numbers
  set(${version_var_prefix}_VERSION ${PACKAGE_XML_version} PARENT_SCOPE)
  set(${version_var_prefix}_VERSION_MAJOR ${PACKAGE_XML_version_major} PARENT_SCOPE)
  set(${version_var_prefix}_VERSION_MINOR ${PACKAGE_XML_version_minor} PARENT_SCOPE)
  set(${version_var_prefix}_VERSION_PATCH ${PACKAGE_XML_version_patch} PARENT_SCOPE)

endfunction()
