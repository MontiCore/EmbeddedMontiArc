<#-- (c) https://github.com/MontiCore/monticore -->
<#if element.inputs?size gte 1>
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
<#if !(tc.architecture.useDgl)>
        ${element.name} = F.identity(${input})
<#else>
        ${element.name} = mx.nd.identity(${input})
</#if>
</#if>
</#if>
