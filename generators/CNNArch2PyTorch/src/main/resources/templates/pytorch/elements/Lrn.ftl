<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.local_response_norm(${input}, size=${element.nsize?c}, alpha=${element.alpha?c}, beta=${element.beta?c}, k=${element.knorm?c})
</#if>