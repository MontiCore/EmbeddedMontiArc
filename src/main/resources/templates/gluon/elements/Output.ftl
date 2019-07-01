<#assign input = element.inputs[0]>
<#assign mode = definition_mode.toString()>
<#if mode == "ARCHITECTURE_DEFINITION">


</#if>
<#if mode == "FORWARD_FUNCTION">
        return ${input}
</#if>
