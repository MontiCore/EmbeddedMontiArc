<#assign dim = element.dim?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = mx.symbol.expand_dims(data = ${element.inputs[0]}, axis=${dim})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = mx.symbol.expand_dims(data = ${element.inputs[0]}, axis=${dim})
</#if>