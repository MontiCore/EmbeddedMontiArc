<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.batch_dot(${tc.join(element.inputs, ", ")})
        <#else>
        ${element.name} = mx.nd.batch.dot(${tc.join(element.inputs, ", ")})
        </#if>
</#if>
