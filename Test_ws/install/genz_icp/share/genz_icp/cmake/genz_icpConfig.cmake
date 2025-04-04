# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_genz_icp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED genz_icp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(genz_icp_FOUND FALSE)
  elseif(NOT genz_icp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(genz_icp_FOUND FALSE)
  endif()
  return()
endif()
set(_genz_icp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT genz_icp_FIND_QUIETLY)
  message(STATUS "Found genz_icp: 0.0.0 (${genz_icp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'genz_icp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${genz_icp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(genz_icp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${genz_icp_DIR}/${_extra}")
endforeach()
