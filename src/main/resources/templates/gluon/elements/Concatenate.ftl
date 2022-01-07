<#-- (c) https://github.com/MontiCore/monticore -->
<#assign axis = (element.axis + 1)?c>
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.concat(${tc.join(element.inputs, ", ")}, dim=${axis})
        <#else>
        ${element.name} = mx.nd.concat(${tc.join(element.inputs, ", ")}, dim=${axis})
        </#if>
</#if>
