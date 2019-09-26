<#assign repeats = element.repeats?c>
<#assign axis = element.axis?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = mx.symbol.repeat(repeats=${repeats}, axis=${axis})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = mx.symbol.repeat(repeats=${repeats}, axis=${axis})
</#if>