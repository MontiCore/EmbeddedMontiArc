<#assign repeats = element.repeats?c>
<#assign axis = element.axis?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = mx.nd.repeat(repeats=${repeats}, axis=${axis})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>