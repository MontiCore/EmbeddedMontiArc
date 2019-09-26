<#if mode == "FORWARD_FUNCTION">
        ${element.name} = mx.symbol.dot(${element.inputs[0]}, ${element.inputs[1]})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = mx.symbol.dot(${element.inputs[0]}, ${element.inputs[1]})
</#if>