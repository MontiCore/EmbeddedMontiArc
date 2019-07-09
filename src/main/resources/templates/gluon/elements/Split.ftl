<#assign input = element.inputs[0]>
<#assign num_outputs = element.numOutputs?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Split(num_outputs=${num_outputs}, axis=1)
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>