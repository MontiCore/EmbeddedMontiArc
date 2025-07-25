<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if element.softmaxOutput>
            ${element.name} = brew.softmax(model, ${input}, '${element.name}')
<#elseif element.logisticRegressionOutput>
            ${element.name} = model.net.Sigmoid(${input}, '${element.name}')
<#elseif element.linearRegressionOutput>
            ${element.name} = ${input}
</#if>

            return ${element.name}
