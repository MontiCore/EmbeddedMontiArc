<#assign input = element.inputs[0]>
<#if element.softmaxOutput>
		${element.name} = brew.softmax(model, ${input}, '${element.name}')
<#elseif element.logisticRegressionOutput>
		${element.name} = mx.symbol.LogisticRegressionOutput(data=${element.inputs[0]}, <#-- TODO: check how to adapt LogisticRegressionOutput -->
            name="${element.name}")
<#elseif element.linearRegressionOutput>
		${element.name} = mx.symbol.LinearRegressionOutput(data=${element.inputs[0]}, <#-- TODO: check how to adapt linearRegressionOutput -->
            name="${element.name}")
</#if>

		return ${element.name}