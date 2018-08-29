<#assign input = element.inputs[0]>
<#assign strideHeight = element.stride[0]>
<#assign strideWidth = element.stride[1]>
<#if element.padding??>
		<#-- TODO: check how to adapt CNNArchLang argument pad_width=${element.padding[0]} -->
</#if>
<#if element.poolType == "max">
	<#if strideHeight == strideWidth>
		${element.name} = brew.max_pool(model, ${input}, '${element.name}', kernel=${element.kernel[0]}, stride=${strideHeight})
	<#else>
		${element.name} = brew.max_pool(model, ${input}, '${element.name}', kernel=${element.kernel[0]}, stride_h=${strideHeight}, stride_w=${strideWidth})
	</#if>
<#elseif element.poolType == "avg">
	<#if strideHeight == strideWidth>
		${element.name} = brew.average_pool(model, ${input}, '${element.name}', kernel=${element.kernel[0]}, stride=${strideHeight})
	<#else>
		${element.name} = brew.average_pool(model, ${input}, '${element.name}', kernel=${element.kernel[0]}, stride_h=${strideHeight}, stride_w=${strideWidth})
	</#if>
</#if>
<#include "OutputShape.ftl">