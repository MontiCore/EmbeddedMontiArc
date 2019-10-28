<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = BroadcastMultiply()
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ", ")})
</#if>