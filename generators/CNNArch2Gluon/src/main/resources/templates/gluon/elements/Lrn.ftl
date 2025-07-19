<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.LRN(data=${input},
            alpha=${element.alpha?c},
            beta=${element.beta?c},
            knorm=${element.knorm?c},
            nsize=${element.nsize?c})
</#if>
