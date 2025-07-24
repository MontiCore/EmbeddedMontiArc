<#-- (c) https://github.com/MontiCore/monticore -->
<#-- This template is not used if the followiing architecture element is an output. See Output.ftl -->
<#assign input = element.inputs[0]>
            ${element.name} = brew.softmax(model, ${input}, '${element.name}')
