<#assign input = element.inputs[0]>
<#assign dim1 = element.axes[0]>
<#assign dim2 = element.axes[1]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = SwapAxes(dim1=${dim1}, dim2=${dim2})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
<#elseif mode == "PYTHON_INLINE">
                    self.${element.name} = SwapAxes(dim1=${dim1}, dim2=${dim2})
</#if>