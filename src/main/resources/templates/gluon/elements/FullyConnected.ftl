<#if element.member == "NONE">
<#assign input = element.inputs[0]>
<#assign units = element.units?c>
<#assign use_bias = element.noBias?string("False","True")>
<#assign flatten = element.flatten?string("True","False")>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.nn.Dense(units=${units}, use_bias=${use_bias}, flatten=${flatten})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
<#elseif element.member == "STATE">
<#if element.inputs?size gte 1>
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = ${input}
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = ${input}
<#elseif mode == "CPP_INLINE">
    ${element.name} = ${input}
</#if>
</#if>
</#if>