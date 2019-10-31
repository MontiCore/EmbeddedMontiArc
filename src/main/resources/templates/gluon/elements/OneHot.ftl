<#assign input = element.inputs[0]>
<#assign size = element.size?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.one_hot(indices=${input}, depth=${size})
</#if>