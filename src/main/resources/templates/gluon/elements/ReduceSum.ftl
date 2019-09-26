<#assign axis = element.axis?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = mx.symbol.sum(data = ${element.inputs[0]}, axis=${axis})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = mx.symbol.sum(data = ${element.inputs[0]}, axis=${axis})
</#if>