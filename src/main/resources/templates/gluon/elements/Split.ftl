<#assign input = element.inputs[0]>
<#assign num_outputs = element.numOutputs?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Split(num_outputs=${num_outputs}, axis=0)
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
<#elseif mode == "PYTHON_INLINE">
<#if input?ends_with("_state_")>
                    ${element.name} = mx.nd.split(data=${input}[0], axis=0, num_outputs=${num_outputs})
<#else>
                    ${element.name} = mx.nd.split(data=${input}, axis=0, num_outputs=${num_outputs})
</#if>
<#elseif mode == "CPP_INLINE">
    ${element.name} = ${input}
</#if>