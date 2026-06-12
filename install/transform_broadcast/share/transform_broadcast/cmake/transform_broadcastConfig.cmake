# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_transform_broadcast_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED transform_broadcast_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(transform_broadcast_FOUND FALSE)
  elseif(NOT transform_broadcast_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(transform_broadcast_FOUND FALSE)
  endif()
  return()
endif()
set(_transform_broadcast_CONFIG_INCLUDED TRUE)

# output package information
if(NOT transform_broadcast_FIND_QUIETLY)
  message(STATUS "Found transform_broadcast: 0.0.0 (${transform_broadcast_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'transform_broadcast' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT transform_broadcast_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(transform_broadcast_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${transform_broadcast_DIR}/${_extra}")
endforeach()
