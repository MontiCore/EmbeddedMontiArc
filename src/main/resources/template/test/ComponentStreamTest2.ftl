<#include "/Common.ftl">
#ifndef ${viewModel.fileNameWithoutExtension?upper_case}
#define ${viewModel.fileNameWithoutExtension?upper_case}

#include "catch.hpp"
#include "../${viewModel.componentName}.h"
#include <iostream>
#include <fstream>
<#list viewModel.streams as stream>
TEST_CASE("${stream.name}", "[${viewModel.componentName}]") {
    ${viewModel.componentName} component;
    std::ofstream myfile;
    myfile.open ("${stream.name}");
    myfile.precision(64);
    component.init();
    <#list stream.checks as check>
        <#list check.inputPortName2Value?keys as portName>
            component.${portName} = ${check.inputPortName2Value[portName]};
        </#list>
        component.execute();
        <#list check.outputPortName2Check?keys as outputPortName>
            <@renderPortCheck outputPortName=outputPortName check=check.outputPortName2Check[outputPortName] />
        </#list>
        <#list stream.outputPortNames as outputPortName>
            std::cout << (component.${outputPortName}) << "\n";
            myfile << "${outputPortName}: " << component.${outputPortName} << "\n";
        </#list>
    </#list>
    myfile.close();
    std::cout << "${stream.name}: success\n";
}
</#list>

#endif

<#macro renderPortCheck outputPortName check>
<#assign portValue="component.${outputPortName}">
<#if helper.isBooleanOutputPortCheck(check)>
    <#if helper.isTrueExpectedCheck(check)>
        REQUIRE( ${portValue} );
    <#else>
        REQUIRE_FALSE( ${portValue} );
    </#if>
<#elseif helper.isRangeOutputPortCheck(check)>
    REQUIRE( ${portValue} >= ${check.lowerBound} );
    REQUIRE( ${portValue} <= ${check.upperBound} );
<#else>
    std::cout << (${portValue}) << "\n";
</#if>
</#macro>
