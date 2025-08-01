<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign dim1 = (element.axes[0] + 1)?c>
<#assign dim2 = (element.axes[1] + 1)?c>
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.swapaxes(${input}, dim1=${dim1}, dim2=${dim2})
        <#else>
        ${element.name} = mx.nd.swapaxes(${input}, dim1=${dim1}, dim2=${dim2})
        </#if>
</#if>
