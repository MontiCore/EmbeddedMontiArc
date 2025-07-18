<#-- (c) https://github.com/MontiCore/monticore -->
<#--@formatter:off-->
# Setting cmake version
cmake_minimum_required (VERSION 2.8)

# Using C++ with flags
set (CMAKE_CXX_FLAGS "-g -std=c++0x")

# Setting project name and description
project(${model.getCompName()})

# Find vsomeip and boost packages
find_package (vsomeip 2.10.0 REQUIRED)
find_package( Boost 1.55 COMPONENTS system thread log REQUIRED )

#include vsomeip and boost directories
include_directories (
    <#noparse>${Boost_INCLUDE_DIR}</#noparse>
    <#noparse>${VSOMEIP_INCLUDE_DIRS}</#noparse>
)

# Linking libraries to target
add_library(SomeIPAdapter_${model.getEscapedCompName()}
SomeIPAdapter_${model.getEscapedCompName()}.cpp
SomeIPAdapter_${model.getEscapedCompName()}.h
)

list(APPEND LIBRARIES ${model.getEscapedCompName()})
list(APPEND LIBRARIES IAdapter_${model.getEscapedCompName()})

# Adding include directory to a target
target_include_directories(SomeIPAdapter_${model.getEscapedCompName()} PUBLIC <#noparse>${LIBRARIES}</#noparse> <#noparse>${INCLUDE_DIRS}</#noparse> <#noparse>${CMAKE_CURRENT_SOURCE_DIR}</#noparse>)

# Linking libraries to target
target_link_libraries(SomeIPAdapter_${model.getEscapedCompName()} PUBLIC vsomeip <#noparse>${Boost_LIBRARIES}</#noparse> <#noparse>${LIBRARIES}</#noparse>)

# Export target to a cmake module file for outside usage
export(TARGETS SomeIPAdapter_${model.getEscapedCompName()} FILE SomeIPAdapter_${model.getEscapedCompName()}.cmake)
