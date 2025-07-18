<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = mx.symbol.Dropout(data=${element.inputs[0]},
            <#--  p=${element.p?c},  -->
            name="${element.name}")
