<#assign input = element.inputs[0]>
<#assign size = element.size?c>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.one_hot(indices=${input}, depth=${size})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = nd.one_hot(indices=${input}, depth=${size})
<#elseif mode == "CPP_INLINE">
    vector<float> ${element.name}(${size}, 0);
    ${element.name}[${input}[0]] = 1;
</#if>