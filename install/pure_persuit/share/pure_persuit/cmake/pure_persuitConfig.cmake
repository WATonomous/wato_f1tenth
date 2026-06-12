# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pure_persuit_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pure_persuit_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pure_persuit_FOUND FALSE)
  elseif(NOT pure_persuit_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pure_persuit_FOUND FALSE)
  endif()
  return()
endif()
set(_pure_persuit_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pure_persuit_FIND_QUIETLY)
  message(STATUS "Found pure_persuit: 0.0.0 (${pure_persuit_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pure_persuit' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT pure_persuit_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pure_persuit_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pure_persuit_DIR}/${_extra}")
endforeach()
