<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>

        ${element.name} = mx.symbol.Reshape(data=${input},
                shape=(${tc.join(element.shape, ",")}),
                name="${element.name}")
<#include "OutputShape.ftl">