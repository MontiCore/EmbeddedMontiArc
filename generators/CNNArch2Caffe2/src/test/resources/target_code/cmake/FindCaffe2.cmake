# Automatically generated file
#
# - Try to find Caffe2
# Once done this will define
#  Caffe2_FOUND - System has Caffe2
#  Caffe2_INCLUDE_DIRS - The Caffe2 include directories
#  Caffe2_LIBRARY_DIRS - The library directories needed to use Caffe2
#  Caffe2_LIBRARIES    - The libraries needed to use Caffe2

find_path(Caffe2_INCLUDE_DIR
        NAMES caffe2
        PATH_SUFFIXES "include" 
        PATHS
        HINTS $ENV{Caffe2_HOME} 
        )
find_library(Caffe2_LIBRARY
        NAMES caffe2
        PATH_SUFFIXES "lib" "lib64" "lib/x86_64-linux-gnu" "examples/lib_win64" "build" "Release" "x64" "x86" 
        PATHS
        HINTS $ENV{Caffe2_HOME} 
        )

include(FindPackageHandleStandardArgs)
# if all listed variables are TRUE
find_package_handle_standard_args(
  Caffe2
  DEFAULT_MSG
  Caffe2_INCLUDE_DIR
  Caffe2_LIBRARY
  )

mark_as_advanced(
  Caffe2_INCLUDE_DIR
  Caffe2_LIBRARY
  )

set(Caffe2_INCLUDE_DIRS ${Caffe2_INCLUDE_DIR})
set(Caffe2_LIBRARIES ${Caffe2_LIBRARY})
