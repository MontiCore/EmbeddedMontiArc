<#assign axis = (element.axis + 1)?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.concat(${tc.join(element.inputs, ", ")}, dim=${axis})
</#if>