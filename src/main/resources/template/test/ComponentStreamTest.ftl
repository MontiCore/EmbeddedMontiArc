<#include "/Common.ftl">
#ifndef ${viewModel.fileNameWithoutExtension?upper_case}
#define ${viewModel.fileNameWithoutExtension?upper_case}

#include "catch.hpp"
#include "../${viewModel.componentName}.h"

<#list viewModel.streams as stream>
TEST_CASE("${stream.name}", "[${viewModel.componentName}]") {
    ${viewModel.componentName} component;
    component.init();
    <#list stream.checks as check>
        <#list check.inputPortName2Value?keys as portName>
            component.${portName} = ${check.inputPortName2Value[portName]};
        </#list>
        component.execute();
        <#list check.outputPortName2Check?keys as outputPortName>
            <@renderPortCheck outputPortName=outputPortName check=check.outputPortName2Check[outputPortName] />
        </#list>
    </#list>
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
</#if>
</#macro>
