<#if element.softmaxOutput>
        pred = brew.fc(model, ${element.inputs[0]}, 'pred', 500, 10)
        ${element.name} = brew.softmax(model, pred, '${element.name}')

<#elseif element.logisticRegressionOutput>
        ${element.name} = mx.symbol.LogisticRegressionOutput(data=${element.inputs[0]},
            name="${element.name}")
<#elseif element.linearRegressionOutput>
        ${element.name} = mx.symbol.LinearRegressionOutput(data=${element.inputs[0]},
            name="${element.name}")
</#if>