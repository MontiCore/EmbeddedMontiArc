<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>

<#if mode == "ARCHITECTURE_DEFINITION">
        self.${element.name} = nn.BatchNorm2d(num_features=${element.element.inputTypes[0].channels?c})
    <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
