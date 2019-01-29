<#assign mode = definition_mode.toString()>
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.nn.Flatten()
            <#include "OutputShape.ftl">
</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>