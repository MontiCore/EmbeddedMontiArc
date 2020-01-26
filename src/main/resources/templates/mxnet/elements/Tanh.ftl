<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = mx.symbol.Activation(data=${element.inputs[0]},
            act_type='tanh',
            name="${element.name}")
