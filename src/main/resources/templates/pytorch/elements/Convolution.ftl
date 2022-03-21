<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign strideVal = element.stride[0]>
<#assign kernelVal = element.kernel[0]>
<#if element.padding??>
        <#assign paddingTupel = tc.join(element.padding, ",")>
        <#if paddingTupel == "0,-1,0,0,0,0,0,0">
                <#assign pad =""> <#-- "valid is default in pytorch"  -->
        <#elseif paddingTupel == "0,0,-1,0,0,0,0,0">
                <#assign pad = ""> <#-- TO DO  no_loss  -->
        <#else>
                <#if strideVal == 1>
                        <#assign pad = ",padding = 'same'">
                <#else>
                        <#assign pad = ",padding = ${(kernelVal/2)?int}">
                </#if>
        </#if>
<#else>
        <#if strideVal == 1>
                <#assign pad = ",padding = 'same'">
        <#else>
                <#assign pad = ",padding = ${(kernelVal/2)?int}">
        </#if>
</#if>

<#if mode == "ARCHITECTURE_DEFINITION">
        self.${element.name} = nn.Conv2d(in_channels=${element.element.inputTypes[0].channels?c}, out_channels=${element.channels?c}, kernel_size=(${tc.join(element.kernel, ",")}), stride=(${tc.join(element.stride, ",")}), bias=${element.noBias?string("False","True")}${pad})
<#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>