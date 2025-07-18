<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>

        ${element.name} = mx.symbol.Embedding(data=${input},
                input_dim=${element.inputDim?c},
                output_dim=${element.outputDim?c},
                name="${element.name}")
<#include "OutputShape.ftl">
