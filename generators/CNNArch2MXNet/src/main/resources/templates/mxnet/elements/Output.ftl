<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = mx.symbol.SoftmaxOutput(data=${element.inputs[0]},
            name="${element.name}")
        
