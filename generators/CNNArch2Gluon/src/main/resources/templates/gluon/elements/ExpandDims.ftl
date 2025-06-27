<#-- (c) https://github.com/MontiCore/monticore -->
<#assign axis = (element.axis + 1)?c>
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.expand_dims(${element.inputs[0]}, axis=${axis})
        <#else>
        ${element.name} = mx.nd.expand.dims(${element.inputs[0]}, axis=${axis})
        </#if>
</#if>
