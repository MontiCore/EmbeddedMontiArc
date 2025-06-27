<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
            ${element.name} = model.net.Sigmoid(${input}, '${element.name}')
