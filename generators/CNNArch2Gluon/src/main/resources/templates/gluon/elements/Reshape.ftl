<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.reshape(${input}, shape=(0,${tc.join(element.shape, ",")}))
        <#else>
        ${element.name} = mx.nd.reshape(${input}, shape=(0,${tc.join(element.shape, ",")}))
        </#if>
</#if>
