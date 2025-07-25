<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign poolType = element.poolType>
<#if poolType == "avg">
    <#assign poolFunctionType = "Avg">
<#else>
    <#assign poolFunctionType = "Max">
</#if>

<#if mode == "ARCHITECTURE_DEFINITION">
        self.${element.name} = nn.Adaptive${poolFunctionType}Pool2d((1, 1))
    <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
        ${element.name} = ${element.name}.reshape(${element.name}.shape[0], -1)
</#if>
