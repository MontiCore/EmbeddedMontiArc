<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = mx.symbol.BatchNorm(data=${element.inputs[0]},
            fix_gamma=${element.fixGamma?string("True","False")},
            name="${element.name}")
