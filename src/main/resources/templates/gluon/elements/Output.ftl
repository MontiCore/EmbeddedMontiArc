<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        outputs.append(${input})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name}_output = ${input}
<#elseif mode == "CPP_INLINE">
    CNN_${element.name} = ${input};
</#if>
