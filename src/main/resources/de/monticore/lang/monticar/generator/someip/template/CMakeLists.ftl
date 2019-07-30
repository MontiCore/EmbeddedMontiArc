<#--@formatter:off-->
# Setting cmake version
cmake_minimum_required (VERSION 2.8)

# Using C++ with flags
set (CMAKE_CXX_FLAGS "-g -std=c++0x")

# Find vsomeip and boost packages
find_package (vsomeip 2.10.0 REQUIRED)
find_package( Boost 1.55 COMPONENTS system thread log REQUIRED )

#include vsomeip and boost directories
include_directories (
    <#noparse>${Boost_INCLUDE_DIR}</#noparse>
    <#noparse>${VSOMEIP_INCLUDE_DIRS}</#noparse>
)

list(APPEND LIBRARIES ${model.getEscapedCompName()})
list(APPEND LIBRARIES IAdapter_${model.getEscapedCompName()})

# Linking libraries to target
add_library(SomeIPAdapter_${model.getEscapedCompName()} SomeIPAdapter_${model.getEscapedCompName()}.cpp)
target_link_libraries(SomeIPAdapter_${model.getEscapedCompName()} vsomeip <#noparse>${Boost_LIBRARIES}</#noparse>)

# Export target to a cmake module file for outside usage
export(TARGETS SomeIPAdapter_${model.getEscapedCompName()} FILE SomeIPAdapter_${model.getEscapedCompName()}.cmake)
