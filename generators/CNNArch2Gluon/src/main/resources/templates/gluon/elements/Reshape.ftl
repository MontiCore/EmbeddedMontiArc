<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.reshape(${input}, shape=(0,${tc.join(element.shape, ",")}))
</#if>