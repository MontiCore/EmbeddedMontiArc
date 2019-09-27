<#assign axis = element.axis?c>
<#assign repeats = element.repeats?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Repeat(repeats=${repeats}, axis=${axis})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${element.inputs[0]})
</#if>