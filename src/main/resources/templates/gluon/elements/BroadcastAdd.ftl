<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = BroadcastAdd()
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ",")})
<#elseif mode == "PYTHON_INLINE">
                    self.${element.name} = BroadcastAdd()
</#if>