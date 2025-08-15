<#-- (c) https://github.com/MontiCore/monticore -->
<#assign flatten = element.element.inputTypes[0].height != 1 || element.element.inputTypes[0].width != 1>
<#assign input = element.inputs[0]>
<#assign inputLayerType = element.element.getInputElement().get()?string>
<#assign inputChannels = element.element.inputTypes[0].channels?c>
<#assign inputHeight = element.element.inputTypes[0].height>
<#assign inputWidth = element.element.inputTypes[0].width>
<#--flatten is not needed since the fc layer applies it automatically-->
<#if inputLayerType?matches("FullyConnected") || (inputHeight == 1 && inputWidth == 1)>
            ${element.name} = brew.fc(model, ${input}, '${element.name}', dim_in=${inputChannels}, dim_out=${element.units?c})
<#else>
            ${element.name} = brew.fc(model, ${input}, '${element.name}', dim_in=${inputChannels} * ${inputHeight} * ${inputWidth}, dim_out=${element.units?c})
</#if>
<#include "OutputShape.ftl">
