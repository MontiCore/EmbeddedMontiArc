<#-- (c) https://github.com/MontiCore/monticore -->
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
