<#-- This template is not used if the following architecture element is an output. See Output.ftl -->
<#assign axis = element.axis?c>
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.softmax(${input}, axis=${axis})
</#if>
