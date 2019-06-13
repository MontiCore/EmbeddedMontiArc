# Setting cmake version
cmake_minimum_required (VERSION 3.12)

# Using C++ version 11 with threads for compiling
set (CMAKE_CXX_FLAGS "-g -std=c++0x")

# Setting project name and description
project(SomeipAdapter VERSION 1.0
                  DESCRIPTION "Someip adapter"
                  LANGUAGES CXX)

# Find packages and include directories
find_package (vsomeip 2.10.0 REQUIRED)
find_package( Boost 1.55 COMPONENTS system thread log REQUIRED )

include_directories (
    ${Boost_INCLUDE_DIR}
    ${VSOMEIP_INCLUDE_DIRS}
)

# Linking libraries to target
add_library(SomeipAdapter_tests_a_compA SomeipAdapter_tests_a_compA.cpp)
target_link_libraries(SomeipAdapter_tests_a_compA vsomeip ${Boost_LIBRARIES})

# Export target to a cmake module file for outside usage
export(TARGETS SomeipAdapter_tests_a_compA FILE SomeipAdapter_tests_a_compA.cmake)
