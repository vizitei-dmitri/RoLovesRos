# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rlr_april_tags_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rlr_april_tags_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rlr_april_tags_FOUND FALSE)
  elseif(NOT rlr_april_tags_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rlr_april_tags_FOUND FALSE)
  endif()
  return()
endif()
set(_rlr_april_tags_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rlr_april_tags_FIND_QUIETLY)
  message(STATUS "Found rlr_april_tags: 0.0.0 (${rlr_april_tags_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rlr_april_tags' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rlr_april_tags_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rlr_april_tags_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rlr_april_tags_DIR}/${_extra}")
endforeach()
