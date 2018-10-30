cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

project(${viewModel.compName} LANGUAGES CXX)

#set cmake module path
set(CMAKE_MODULE_PATH ${r"${CMAKE_MODULE_PATH}"} ${r"${CMAKE_CURRENT_SOURCE_DIR}"}/cmake)

# add dependencies
<#list viewModel.moduleDependencies as var>
find_package(${var.packageName} <#if var.required>REQUIRED<#else></#if>)
<#if var.findPath>set(INCLUDE_DIRS ${r"${INCLUDE_DIRS}"} ${r"${"}${var.packageName}${r"_INCLUDE_DIRS}"})</#if>
<#if var.findLibrary>set(LIBS ${r"${LIBS}"} ${r"${"}${var.packageName}${r"_LIBRARIES}"})</#if>
</#list>

# additional commands
<#list viewModel.cmakeCommandList as cmd>
${cmd}
</#list>

# create static library
include_directories(${r"${INCLUDE_DIRS}"})
add_library(${viewModel.compName} ${viewModel.compName}.h)
target_include_directories(${viewModel.compName} PUBLIC ${r"${CMAKE_CURRENT_SOURCE_DIR}"} ${r"${INCLUDE_DIRS}"})
target_link_libraries(${viewModel.compName} PUBLIC ${r"${LIBS}"})
set_target_properties(${viewModel.compName} PROPERTIES LINKER_LANGUAGE CXX)

# export cmake project
export(TARGETS ${viewModel.compName} FILE ${viewModel.compName}.cmake)

# additional commands end
<#list viewModel.cmakeCommandListEnd as cmd>
${cmd}
</#list>
