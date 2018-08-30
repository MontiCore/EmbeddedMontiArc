<#assign input = element.inputs[0]>
<#assign strideHeight = element.stride[0]>
<#assign strideWidth = element.stride[1]>
<#assign kernelHeight = element.kernel[0]>
<#assign kernelWidth = element.kernel[1]>
<#if element.padding??>
		<#-- TODO: check how to adapt CNNArchLang argument pad_width=${element.padding[0]} -->
</#if>
<#if strideHeight == strideWidth>
	<#assign strideParameter = "stride=${strideHeight}">
<#else>
	<#assign strideParameter = "stride_h=${strideHeight}, stride_w=${strideWidth}">
</#if>
<#if kernelHeight == kernelWidth>
	<#assign kernelParameter = "kernel=${kernelHeight}">
<#else>
	<#assign kernelParameter = "kernel_h=${kernelHeight}, kernel_w=${kernelWidth}">
</#if>
<#if element.poolType == "max">
		${element.name} = brew.max_pool(model, ${input}, '${element.name}', ${kernelParameter}, ${strideParameter})
<#elseif element.poolType == "avg">
		${element.name} = brew.average_pool(model, ${input}, '${element.name}', ${kernelParameter}, ${strideParameter})
</#if>
<#include "OutputShape.ftl">