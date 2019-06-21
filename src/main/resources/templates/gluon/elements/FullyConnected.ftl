<#assign flatten = element.element.inputTypes[0].height != 1 || element.element.inputTypes[0].width != 1>
<#assign input = element.inputs[0]>
<#assign units = element.units?c>
<#assign use_bias = element.noBias?string("False","True")>
<#if mode == "ARCHITECTURE_DEFINITION">
<#if flatten>
            self.${element.name}flatten = gluon.nn.Flatten()
</#if>
            self.${element.name} = gluon.nn.Dense(units=${units}, use_bias=${use_bias})
            <#include "OutputShape.ftl">
</#if>
<#if mode == "FORWARD_FUNCTION">
<#if flatten>
        ${element.name}flatten_ = self.${element.name}flatten(${input})
        <#assign input = element.name + "flatten_">
</#if>
        ${element.name} = self.${element.name}(${input})
</#if>