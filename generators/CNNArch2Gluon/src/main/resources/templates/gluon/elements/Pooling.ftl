<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign poolType = element.poolType>
<#assign poolSize = "(" + tc.join(element.kernel, ",") + ")">
<#assign strides = "(" + tc.join(element.stride, ",") + ")">
<#if poolType == "avg">
    <#assign poolFunctionType = "Avg">
<#else>
    <#assign poolFunctionType = "Max">
</#if>
<#if mode == "ARCHITECTURE_DEFINITION">
<#if element.padding??>
            self.${element.name}padding = Padding(padding=(${tc.join(element.padding, ",")}))
</#if>
            self.${element.name} = gluon.nn.${poolFunctionType}Pool2D(
                pool_size=${poolSize},
                strides=${strides})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
<#if element.padding??>
        ${element.name}padding = self.${element.name}padding(${input})
        <#assign input = element.name + "padding">
</#if>
        ${element.name} = self.${element.name}(${input})
</#if>
