<#assign input = element.inputs[0]>
<#assign mode = definition_mode.toString()>
<#assign poolType = element.poolType>
<#if poolType == "avg">
    <#assign poolFunctionType = "Avg">
<#else>
    <#assign poolFunctionType = "Max">
</#if>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.nn.Global${poolFunctionType}Pool2D()
            <#include "OutputShape.ftl">
</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>