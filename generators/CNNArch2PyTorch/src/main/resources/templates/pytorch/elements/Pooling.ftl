<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign strideHeight = element.stride[0]>
<#assign strideWidth = element.stride[1]>
<#assign kernelHeight = element.kernel[0]>
<#assign kernelWidth = element.kernel[1]>
<#if element.padding??>
	<#assign paddingTupel = tc.join(element.padding, ",")>
	<#if paddingTupel == "0,-1,0,0,0,0,0,0">
		<#assign pad = "">
	<#elseif paddingTupel == "0,0,-1,0,0,0,0,0">
		<#assign pad = ""> <#-- TO DO  no_loss  -->
	<#else>
		<#assign pad = ",padding = ${(kernelHeight/2)?int}">  <#-- TO DO  check later  -->
	</#if>
<#else>
	<#assign pad = "">
</#if>
<#if strideHeight == strideWidth>
	<#assign strideParameter = "stride=${strideHeight}">
<#else>
	<#assign strideParameter = "stride=(${strideHeight}, ${strideWidth})">
</#if>
<#if kernelHeight == kernelWidth>
	<#assign kernelParameter = "kernel_size=${kernelHeight}">
<#else>
	<#assign kernelParameter = "kernel_size=(${kernelHeight}, ${kernelWidth})">
</#if>
<#if mode == "ARCHITECTURE_DEFINITION">
<#if element.poolType == "max">
        self.${element.name} = nn.MaxPool2d(${kernelParameter}, ${strideParameter}${pad})
<#elseif element.poolType == "avg">
        self.${element.name} = nn.AvgPool2d(${kernelParameter}, ${strideParameter}${pad})
</#if>
<#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>