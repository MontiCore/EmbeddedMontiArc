<#-- (c) https://github.com/MontiCore/monticore -->
# Automatically generated file
#
# - Try to find ${viewModel.packageName}
# Once done this will define
#  ${viewModel.packageName}_FOUND - System has ${viewModel.packageName}
#  ${viewModel.packageName}_INCLUDE_DIRS - The ${viewModel.packageName} include directories
#  ${viewModel.packageName}_LIBRARY_DIRS - The library directories needed to use ${viewModel.packageName}
#  ${viewModel.packageName}_LIBRARIES    - The libraries needed to use ${viewModel.packageName}

<#if viewModel.findPath>
find_path(${viewModel.packageName}_INCLUDE_DIR
        NAMES ${viewModel.includeName}
        PATH_SUFFIXES "include"
        PATHS
        <#list viewModel.includePaths as var>
        "${var}"
        </#list>
        HINTS $ENV{${viewModel.packageName}_HOME}
        )
</#if>
<#if viewModel.findLibrary>
find_library(${viewModel.packageName}_LIBRARY
        NAMES ${viewModel.libName}
        PATH_SUFFIXES "lib" "lib64" "lib/x86_64-linux-gnu" "examples/lib_win64" "build" "Release"
        PATHS
        <#list viewModel.libPaths as var>
        "${var}"
        </#list>
        HINTS $ENV{${viewModel.packageName}_HOME}
        )
</#if>

include(FindPackageHandleStandardArgs)
# if all listed variables are TRUE
find_package_handle_standard_args(
  ${viewModel.packageName}
  DEFAULT_MSG
  <#if viewModel.findPath>${viewModel.packageName}_INCLUDE_DIR</#if>
  <#if viewModel.findLibrary>${viewModel.packageName}_LIBRARY</#if>
  )

mark_as_advanced(
  <#if viewModel.findPath>${viewModel.packageName}_INCLUDE_DIR</#if>
  <#if viewModel.findLibrary>${viewModel.packageName}_LIBRARY</#if>
  )

<#if viewModel.findPath>set(${viewModel.packageName}_INCLUDE_DIRS ${r"${"}${viewModel.packageName}${r"_INCLUDE_DIR}"})</#if>
<#if viewModel.findLibrary>set(${viewModel.packageName}_LIBRARIES ${r"${"}${viewModel.packageName}${r"_LIBRARY}"})</#if>
