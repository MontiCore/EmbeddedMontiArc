<#assign input = element.inputs[0]>
<#assign strideHeight = element.stride[0]>
<#assign strideWidth = element.stride[1]>
<#if element.padding??>  <#-- Check wheather padding null is. -->
		<#-- TODO: check how to adapt CNNArchLang argument pad_width=${element.padding[0]} -->
</#if>
<#if input = tc.architectureInputs[0]>	<#-- TODO: CHECK COMPARISON -->
	<#if strideHeight == strideWidth>
		${element.name} = brew.conv(model, '${input}', '${element.name}', dim_in=1, dim_out=${element.channels?c}, kernel=${element.kernel[0]}, stride=${strideHeight})
	<#else>
		${element.name} = brew.conv(model, '${input}', '${element.name}', dim_in=1, dim_out=${element.channels?c}, kernel=${element.kernel[0]}, stride_h=${strideHeight}, stride_w=${strideWidth})
	</#if>
<#else>
	<#if strideHeight == strideWidth>
		${element.name} = brew.conv(model, ${input}, '${element.name}', dim_in=${element.element.inputTypes[0].channels?c}, dim_out=${element.channels?c}, kernel=${element.kernel[0]}, stride=${strideHeight})
	<#else>
		${element.name} = brew.conv(model, ${input}, '${element.name}', dim_in=${element.element.inputTypes[0].channels?c}, dim_out=${element.channels?c}, kernel=${element.kernel[0]}, stride_h=${strideHeight}, stride_w=${strideWidth})
	</#if>
</#if>
		<#-- TODO: check how to adapt CNNArchLang argument no_bias=${element.noBias?string("True","False")} -->
<#include "OutputShape.ftl">
		# Yeverino input layer name: ${element.inputs[0]}
		# Yeverino input/previous layer dim_out: ${element.element.inputTypes[0].channels?c}
		# Yeverino current layer dim_out: ${element.element.outputTypes[0].channels?c}
