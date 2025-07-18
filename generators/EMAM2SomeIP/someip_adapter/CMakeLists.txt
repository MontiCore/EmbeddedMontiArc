# Setting cmake version
cmake_minimum_required (VERSION 2.8)

# Using C++ with flags
set (CMAKE_CXX_FLAGS "-g -std=c++0x")

# Find vsomeip and boost packages
find_package (vsomeip 2.10.0 REQUIRED)
find_package( Boost 1.55 COMPONENTS system thread log REQUIRED )

#include vsomeip and boost directories
include_directories (
    ${Boost_INCLUDE_DIR}
    ${VSOMEIP_INCLUDE_DIRS}
)

# Linking libraries to target
add_library(SomeIPAdapter_tests_a_compA SomeIPAdapter_tests_a_compA.cpp
SomeIPAdapter_tests_a_compA.cpp
SomeIPAdapter_tests_a_compA.h
tests_a_compA.cpp
tests_a_compA.h
)

list(APPEND LIBRARIES tests_a_compA)
#list(APPEND LIBRARIES IAdapter_tests_a_compA)

target_link_libraries(SomeIPAdapter_tests_a_compA vsomeip ${Boost_LIBRARIES} ${LIBRARIES})

target_include_directories(SomeIPAdapter_tests_a_compA PUBLIC ${INCLUDE_DIRS})

# Export target to a cmake module file for outside usage
export(TARGETS SomeIPAdapter_tests_a_compA FILE SomeIPAdapter_tests_a_compA.cmake)
