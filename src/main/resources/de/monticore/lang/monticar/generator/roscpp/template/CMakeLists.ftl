<#-- (c) https://github.com/MontiCore/monticore -->
cmake_minimum_required(VERSION 3.5)
project (${model.name})
set (CMAKE_CXX_STANDARD 14)
set (AMENT_CMAKE_UNINSTALL_TARGET FALSE)

<#list model.getPackages() as pack>
<#if !model.getExcludeFindPackages()?seq_contains(pack)>
find_package(${pack} REQUIRED)
</#if>
</#list>

add_library(${model.name} ${model.name}.cpp)

list(APPEND LIBRARIES ${model.compName})
list(APPEND LIBRARIES IAdapter_${model.compName})
<#list model.getPackages() as lib>
list(APPEND LIBRARIES ${r"${"}${lib}_LIBRARIES})
</#list>
target_link_libraries(${model.name} <#noparse>${LIBRARIES}</#noparse>)

list(APPEND INCLUDE_DIRS <#noparse>${CMAKE_CURRENT_SOURCE_DIR}</#noparse>)
<#list model.getPackages() as incl>
list(APPEND INCLUDE_DIRS ${r"${"}${incl}_INCLUDE_DIRS})
</#list>
target_include_directories(${model.name} PUBLIC <#noparse>${INCLUDE_DIRS}</#noparse>)

export(TARGETS ${model.name} FILE ${model.name}.cmake)
