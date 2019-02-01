<#assign input = element.inputs[0]>
<#if element.softmaxOutput>
    		${element.name} = brew.softmax(model, ${input}, '${element.name}')
<#elseif element.logisticRegressionOutput>
    		${element.name} = model.net.Sigmoid(${input}, '${element.name}')
<#elseif element.linearRegressionOutput>
    		<#--Don't add L2 loss here but within the function "add_training_operators" from CNNCreator.ftl-->
    		${element.name} = ${input}
</#if>

    		return ${element.name}