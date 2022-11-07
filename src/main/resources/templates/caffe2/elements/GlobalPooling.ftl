<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if element.poolType == "max">
            ${element.name} = brew.max_pool(model, ${input}, '${element.name}', global_pooling=True)
<#elseif element.poolType == "avg">
            ${element.name} = brew.average_pool(model, ${input}, '${element.name}', global_pooling=True)
</#if>
<#include "OutputShape.ftl">
