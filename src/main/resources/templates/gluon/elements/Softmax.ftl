<#-- (c) https://github.com/MontiCore/monticore -->
<#-- This template is not used if the following architecture element is an output. See Output.ftl -->
<#if element.axis == -1>
<#assign axis = element.axis?c>
<#else>
<#assign axis = (element.axis + 1)?c>
</#if>
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.softmax(${input}, axis=${axis})
</#if>
