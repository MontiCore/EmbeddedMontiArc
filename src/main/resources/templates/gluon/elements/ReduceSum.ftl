<#assign axis = element.axis?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = ReduceSum(axis=${axis})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${element.inputs[0]})
</#if>