<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign size = element.size?c>
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.one_hot(indices=${input}, depth=${size})
        <#else>
        ${element.name} = mx.nd.one.hot(indices=${input}, depth=${size})
        </#if>
</#if>

