<#-- (c) https://github.com/MontiCore/monticore -->
<#assign axis = (element.axis != -1)?then((element.axis + 1)?c, 'None')>
<#assign repeats = element.repeats?c>
<#if mode == "FORWARD_FUNCTION">
        <#if !(tc.architecture.useDgl)>
        ${element.name} = F.repeat(${element.inputs[0]}, repeats=${repeats}, axis=${axis})
        <#else>
        ${element.name} = mx.nd.repeat(${element.inputs[0]}, repeats=${repeats}, axis=${axis})
        </#if>
</#if>
