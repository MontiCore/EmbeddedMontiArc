<#assign axis = (element.axis != -1)?then((element.axis + 1)?c, 'None')>
<#assign repeats = element.repeats?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.repeat(${element.inputs[0]}, repeats=${repeats}, axis=${axis})
</#if>