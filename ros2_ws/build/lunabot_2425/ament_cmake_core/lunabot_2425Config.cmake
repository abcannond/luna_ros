# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_lunabot_2425_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED lunabot_2425_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(lunabot_2425_FOUND FALSE)
  elseif(NOT lunabot_2425_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(lunabot_2425_FOUND FALSE)
  endif()
  return()
endif()
set(_lunabot_2425_CONFIG_INCLUDED TRUE)

# output package information
if(NOT lunabot_2425_FIND_QUIETLY)
  message(STATUS "Found lunabot_2425: 0.0.0 (${lunabot_2425_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'lunabot_2425' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT lunabot_2425_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(lunabot_2425_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${lunabot_2425_DIR}/${_extra}")
endforeach()
