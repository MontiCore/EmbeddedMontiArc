<#if mode == "FORWARD_FUNCTION">
        ${element.name} = ${tc.join(element.inputs, " + ")}
</#if>