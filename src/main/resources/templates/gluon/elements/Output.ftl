<#-- (c) https://github.com/MontiCore/monticore -->
<#if element.inputs?size gte 1>
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.identity(${input})
</#if>
</#if>
