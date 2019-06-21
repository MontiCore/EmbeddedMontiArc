<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Concatenate(dim=1)
            <#include "OutputShape.ftl">
</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ", ")})
</#if>