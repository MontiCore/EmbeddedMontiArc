<#if element.member == "NONE">
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.rnn.LSTM(hidden_size=${element.units?c},
                num_layers=${element.layers?c},
                bidirectional=${element.bidirectional?string("True", "False")}
                layout='NTC')
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
<#if element.isVariable()>
        ${element.name}, ${element.element.name}_state_ = self.${element.name}(${input}, ${element.element.name}_state_)
<#else>
        ${element.name} = self.${element.name}(${input})
</#if>
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