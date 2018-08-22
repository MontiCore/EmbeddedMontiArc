<#assign flatten = element.element.inputTypes[0].height != 1 || element.element.inputTypes[0].width != 1>
<#assign input = element.inputs[0]>
<#if flatten>
        ${element.name} = mx.symbol.flatten(data=${input}) #TODO: check how to adapt CNNArchLang flatten
<#assign input = element.name>
</#if>
        ${element.name} = brew.fc(model, ${input}, '${element.name}', dim_in=50 * 4 * 4, dim_out=${element.units?c})

        #TODO: check how to adapt CNNArchLang argument no_bias=${element.noBias?string("True","False")}
