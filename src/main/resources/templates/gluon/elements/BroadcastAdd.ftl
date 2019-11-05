<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.broadcast_add(${tc.join(element.inputs, ",")})
<#elseif mode == "PYTHON_INLINE">
                    self.${element.name} = mx.nd.broadcast_add(${tc.join(element.inputs, ",")})
</#if>