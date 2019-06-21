<#assign input = element.inputs[0]>
<#assign size = element.size>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = OneHot(size=${size})
            <#include "OutputShape.ftl">
</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
