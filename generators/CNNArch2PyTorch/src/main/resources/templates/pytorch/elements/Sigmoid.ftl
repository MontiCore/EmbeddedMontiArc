<#-- (c) https://github.com/MontiCore/monticore -->
<#-- This template is not used if the following architecture element is an output. See Output.ftl -->

<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.sigmoid(${input})
</#if>