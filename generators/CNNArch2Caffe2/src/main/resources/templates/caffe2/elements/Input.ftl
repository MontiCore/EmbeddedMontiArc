<#-- (c) https://github.com/MontiCore/monticore -->
<#assign channelIndex = element.element.outputTypes[0].channelIndex + 1>
<#assign heightIndex = element.element.outputTypes[0].heightIndex + 1>
<#assign widthIndex = element.element.outputTypes[0].widthIndex + 1>
<#assign indexList = []>
<#if channelIndex != 0><#assign indexList = indexList + [channelIndex]></#if>
<#if heightIndex != 0><#assign indexList = indexList + [heightIndex]></#if>
<#if widthIndex != 0><#assign indexList = indexList + [widthIndex]></#if>
<#assign dimensions = element.element.outputTypes[0].dimensions>
            ${element.name} = data
<#include "OutputShape.ftl">
<#if heightIndex != channelIndex + 1 || widthIndex != heightIndex + 1>
            ${element.name} = model.net.Transpose(${element.name}, '${element.name}', axes=[0,${tc.join(indexList, ",")}])

</#if>
<#if indexList?size != 3>
            ${element.name}, _ = model.net.Reshape('${element.name}', ['${element.name}', '${element.name}_old_shape'],
                shape=(0,${element.element.outputTypes[0].channels?c},${element.element.outputTypes[0].height?c},${element.element.outputTypes[0].width?c}))
</#if>
