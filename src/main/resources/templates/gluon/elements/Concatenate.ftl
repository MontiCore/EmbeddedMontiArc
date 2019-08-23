<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Concatenate(dim=1)
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ", ")})
</#if>
