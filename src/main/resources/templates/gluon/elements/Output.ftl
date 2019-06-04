<#assign input = element.inputs[0]>
<#assign mode = definition_mode.toString()>
<#if mode == "ARCHITECTURE_DEFINITION">
    <#if element.softmaxOutput>
        self.last_layer = 'softmax'
    <#elseif element.logisticRegressionOutput>
        self.last_layer = 'sigmoid'
    <#elseif element.linearRegressionOutput>
        self.last_layer = 'linear'
    </#if>
</#if>
<#if mode == "FORWARD_FUNCTION">
        return ${input}
</#if>
