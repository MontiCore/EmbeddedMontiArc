<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign num_outputs = element.numOutputs?c>

<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.split(${input}, axis=1, num_outputs=${num_outputs})
        <#else>
        ${element.name} = mx.nd.split(${input}, axis=1, num_outputs=${num_outputs})
        </#if>
</#if>
