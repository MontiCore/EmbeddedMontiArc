<#assign input = element.inputs[0]>
<#assign num_outputs = element.numOutputs?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.split(${input}, axis=1, num_outputs=${num_outputs})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = mx.nd.split(data=${input}, axis=1, num_outputs=${num_outputs})
<#elseif mode == "CPP_INLINE">
    ${element.name} = ${input}  // TODO: Implement
</#if>