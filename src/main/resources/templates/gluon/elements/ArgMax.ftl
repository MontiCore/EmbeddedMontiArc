<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.ndarray.argmax(${input}, keepdims=True)
</#if>