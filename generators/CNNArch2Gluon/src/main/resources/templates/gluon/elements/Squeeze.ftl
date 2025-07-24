<#-- (c) https://github.com/MontiCore/monticore -->
<#assign axis = (element.axis != -1)?then((element.axis + 1)?c, 'None')>
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.squeeze(${element.inputs[0]}, axis=${axis})
        <#else>
        ${element.name} = mx.nd.squeeze(${element.inputs[0]}, axis=${axis})
        </#if>
</#if>
