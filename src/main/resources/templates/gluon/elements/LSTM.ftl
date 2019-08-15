<#if element.member == "NONE">
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.rnn_${element.element.name} = gluon.rnn.LSTM(hidden_size=${element.units?c}, num_layers=${element.layers?c}, layout='NTC')
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name}, ${element.element.name}_state_ = self.rnn_${element.element.name}(${input}, ${element.element.name}_state_)
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