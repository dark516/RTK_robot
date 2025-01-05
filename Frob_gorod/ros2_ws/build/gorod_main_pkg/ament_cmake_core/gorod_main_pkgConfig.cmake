# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gorod_main_pkg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gorod_main_pkg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gorod_main_pkg_FOUND FALSE)
  elseif(NOT gorod_main_pkg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gorod_main_pkg_FOUND FALSE)
  endif()
  return()
endif()
set(_gorod_main_pkg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gorod_main_pkg_FIND_QUIETLY)
  message(STATUS "Found gorod_main_pkg: 0.1.0 (${gorod_main_pkg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gorod_main_pkg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT gorod_main_pkg_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gorod_main_pkg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gorod_main_pkg_DIR}/${_extra}")
endforeach()
