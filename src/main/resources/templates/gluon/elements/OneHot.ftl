<#assign input = element.inputs[0]>
<#assign size = element.size>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = OneHot(size=${element.element.outputTypes[0].dimensions[0]})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = nd.one_hot(indices=${input}, depth=${element.element.outputTypes[0].dimensions[0]})
<#elseif mode == "CPP_INLINE">
    vector<float> ${element.name}(${element.element.outputTypes[0].dimensions[0]}, 0);
    ${element.name}[${input}[0]] = 1;
</#if>