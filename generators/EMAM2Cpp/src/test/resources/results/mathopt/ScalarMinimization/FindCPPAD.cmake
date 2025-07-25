# Automatically generated file
#
# - Try to find CPPAD
# Once done this will define
#  CPPAD_FOUND - System has CPPAD
#  CPPAD_INCLUDE_DIRS - The CPPAD include directories
#  CPPAD_LIBRARY_DIRS - The library directories needed to use CPPAD
#  CPPAD_LIBRARIES    - The libraries needed to use CPPAD

find_path(CPPAD_INCLUDE_DIR
        NAMES cppad/ipopt/solve.hpp
        PATH_SUFFIXES "include" 
        PATHS
        HINTS $ENV{CPPAD_HOME} 
        )

include(FindPackageHandleStandardArgs)
# if all listed variables are TRUE
find_package_handle_standard_args(
  CPPAD
  DEFAULT_MSG
  CPPAD_INCLUDE_DIR
  
  )

mark_as_advanced(
  CPPAD_INCLUDE_DIR
  )

set(CPPAD_INCLUDE_DIRS ${CPPAD_INCLUDE_DIR})
