<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Reshape(shape=(${tc.join(element.shape, ",")}))
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
<#elseif mode == "PYTHON_INLINE">
                    self.${element.name} = Reshape(shape=${shape})
</#if>