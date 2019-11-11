<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.reshape(${input}, shape=(${tc.join(element.shape, ",")}))
</#if>