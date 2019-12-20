<#if mode == "FORWARD_FUNCTION">
        ${element.name} = gluon.nn.slice(${tc.join(element.inputs, ", ")})
</#if>
