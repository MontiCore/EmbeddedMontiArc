<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = ${input}
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = ${input}
<#elseif mode == "CPP_INLINE">
    ${element.name} = ${input};
</#if>
