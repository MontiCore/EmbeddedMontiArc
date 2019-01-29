<#assign input = element.inputs[0]>
<#assign mode = definition_mode.toString()>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.LRN(data=${input},
            alpha=${element.alpha?c},
            beta=${element.beta?c},
            knorm=${element.knorm?c},
            nsize=${element.nsize?c})
</#if>