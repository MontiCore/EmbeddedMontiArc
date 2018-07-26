@ECHO OFF

<#list cppIncludes>
set TMP_CPLUS_INCLUDE_PATH=%CPLUS_INCLUDE_PATH%
    <#items as item>
set CPLUS_INCLUDE_PATH=${item};%CPLUS_INCLUDE_PATH%
    </#items>
</#list>

${gppCommand} -std=c++11 -DCATCH_CONFIG_MAIN=1 -DARMA_DONT_USE_WRAPPER "test/tests_main.cpp" -o ${execName}

<#list cppIncludes>
set CPLUS_INCLUDE_PATH=%TMP_CPLUS_INCLUDE_PATH%
    <#items as item>
    </#items>
</#list>