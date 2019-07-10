<#assign input = element.inputs[0]>
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
<#elseif mode == "FORWARD_FUNCTION">
        outputs.append(${input})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name}_output = ${input}
<#elseif mode == "CPP_INLINE">
    CNN_${element.name} = ${input};
</#if>
