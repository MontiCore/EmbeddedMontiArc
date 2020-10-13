<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.broadcast_mul(${tc.join(element.inputs, ", ")})
</#if>