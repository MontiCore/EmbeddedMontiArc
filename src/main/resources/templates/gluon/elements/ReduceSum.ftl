<#assign axis = (element.axis + 1)?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.sum(${element.inputs[0]}, axis=${axis})
</#if>