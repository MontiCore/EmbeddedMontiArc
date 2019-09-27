<#assign dim = element.dim?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = ExpandDims(dim=${dim})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${element.inputs[0]})
</#if>