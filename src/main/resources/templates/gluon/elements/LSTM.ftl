<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = CustomLSTM(hidden_size=${element.units?c},
                num_layers=${element.layers?c},
                dropout=${element.dropout?c},
                bidirectional=${element.bidirectional?string("True", "False")})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
<#if element.isVariable()>
        ${element.name}, ${element.element.name}_state_0_, ${element.element.name}_state_1_ = self.${element.name}(${input}, ${element.element.name}_state_0_, ${element.element.name}_state_1_)
<#else>
        ${element.name} = self.${element.name}(${input})
</#if>
</#if>

