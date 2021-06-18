# Automatically generated file
#
# - Try to find Ipopt
# Once done this will define
#  Ipopt_FOUND - System has Ipopt
#  Ipopt_INCLUDE_DIRS - The Ipopt include directories
#  Ipopt_LIBRARY_DIRS - The library directories needed to use Ipopt
#  Ipopt_LIBRARIES    - The libraries needed to use Ipopt

find_path(Ipopt_INCLUDE_DIR
        NAMES coin/IpNLP.hpp
        PATH_SUFFIXES "include" 
        PATHS
        HINTS $ENV{Ipopt_HOME} 
        )
find_library(Ipopt_LIBRARY
        NAMES ipopt
        PATH_SUFFIXES "lib" "lib64" "lib/x86_64-linux-gnu" "examples/lib_win64" "build" "Release" "x64" "x86" 
        PATHS
        HINTS $ENV{Ipopt_HOME} 
        )

include(FindPackageHandleStandardArgs)
# if all listed variables are TRUE
find_package_handle_standard_args(
  Ipopt
  DEFAULT_MSG
  Ipopt_INCLUDE_DIR
  Ipopt_LIBRARY
  )

mark_as_advanced(
  Ipopt_INCLUDE_DIR
  Ipopt_LIBRARY
  )

set(Ipopt_INCLUDE_DIRS ${Ipopt_INCLUDE_DIR})
set(Ipopt_LIBRARIES ${Ipopt_LIBRARY})
