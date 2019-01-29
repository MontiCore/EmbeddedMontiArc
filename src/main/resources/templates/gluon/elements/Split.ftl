<#assign input = element.inputs[0]>
<#assign mode = definition_mode.toString()>
<#assign num_outputs = element.numOutputs?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Split(num_outputs=${num_outputs}, axis=1)
            <#include "OutputShape.ftl">
</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>