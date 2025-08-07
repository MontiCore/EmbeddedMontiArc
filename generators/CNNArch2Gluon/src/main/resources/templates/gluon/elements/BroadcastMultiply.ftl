<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.broadcast_mul(${tc.join(element.inputs, ", ")})
        <#else>
        ${element.name} = mx.nd.broadcast.mul(${tc.join(element.inputs, ", ")})
        </#if>
</#if>
