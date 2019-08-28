<#assign input = element.inputs[0]>
<#assign units = element.units?c>
<#assign use_bias = element.noBias?string("False","True")>
<#assign flatten = element.flatten?string("True","False")>
<#if mode == "ARCHITECTURE_DEFINITION">
        <#if (element.partOfUnroll && false)>
            ${element.name} = Net_1().${element.name}(${input})
        <#else>
            self.${element.name} = gluon.nn.Dense(units=${units}, use_bias=${use_bias}, flatten=${flatten})
        </#if>
        <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
    <#if (element.partOfUnroll && false)>
        ${element.name} = Net_1().${element.name}(${input})
    <#else>
        ${element.name} = self.${element.name}(${input})
    </#if>
</#if>