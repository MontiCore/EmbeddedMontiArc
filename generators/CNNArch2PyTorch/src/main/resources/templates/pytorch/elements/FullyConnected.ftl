<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign inputLayerType = element.element.getInputElement().get()?string>
<#assign inputChannels = element.element.inputTypes[0].channels?c>
<#assign inputHeight = element.element.inputTypes[0].height>
<#assign inputWidth = element.element.inputTypes[0].width>

<#if mode == "ARCHITECTURE_DEFINITION">
<#if inputLayerType?matches("FullyConnected") || (inputHeight == 1 && inputWidth == 1)>
        self.${element.name} = nn.Linear(in_features=${inputChannels}, out_features=${element.units?c},bias=${element.noBias?string("False","True")})
<#else>
        self.${element.name} = nn.Linear(in_features=${inputChannels} * ${inputHeight} * ${inputWidth}, out_features=${element.units?c},bias=${element.noBias?string("False","True")})
</#if>
<#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
<#if inputLayerType?matches("FullyConnected") || (inputHeight == 1 && inputWidth == 1)>
        ${element.name} = self.${element.name}(${input})
<#else>
        ${element.name} = ${input}.reshape(${input}.shape[0], -1)
        ${element.name} = self.${element.name}(${element.name})
</#if>
</#if>