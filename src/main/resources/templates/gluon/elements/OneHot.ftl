<#assign input = element.inputs[0]>
<#assign size = element.size?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = OneHot(size=${size})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = nd.one_hot(indices=${input}, depth=${size})
<#elseif mode == "CPP_INLINE">
    vector<float> ${element.name}(${size}, 0);
    ${element.name}[${input}[0]] = 1;
</#if>