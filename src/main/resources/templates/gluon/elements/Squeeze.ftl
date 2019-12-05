<#assign axis = (element.axis != -1)?then((element.axis + 1)?c, 'None')>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.squeeze(${element.inputs[0]}, axis=${axis})
</#if>