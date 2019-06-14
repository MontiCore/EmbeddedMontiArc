<#assign input = element.inputs[0]>
<#assign mode = definition_mode.toString()>
<#if mode == "ARCHITECTURE_DEFINITION">
    <#if element.softmaxOutput>
        self.last_layers['${element.name}'] = 'softmax'
    <#elseif element.logisticRegressionOutput>
        self.last_layers['${element.name}'] = 'sigmoid'
    <#elseif element.linearRegressionOutput>
        self.last_layers['${element.name}'] = 'linear'
    <#elseif element.oneHotOutput>
        self.last_layers['${element.name}'] = 'softmax'
    </#if>
</#if>
<#if mode == "FORWARD_FUNCTION">
    <#if tc.architectureOutputs?size gt 1>
        outputs.append(${input})
    <#else>
        return ${input}
    </#if>
</#if>
