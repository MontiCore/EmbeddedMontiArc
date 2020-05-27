<#-- (c) https://github.com/MontiCore/monticore -->
<#assign inputQueries = element.inputs[0]>
<#assign inputKeys = element.inputs[1]>
<#assign inputValues = element.inputs[2]>
<#assign scaleFactor = element.scaleFactor?c>
<#assign numHeads = element.numHeads?c>
<#assign dimKeys = element.dimKeys?c>
<#assign dimValues = element.dimValues?c>
<#assign useProjBias = element.useProjBias?string("True", "False")>
<#if mode == "ARCHITECTURE_DEFINITION">
<#assign dimModel = 1>
<#list element.element.outputTypes[0].dimensions as dim>
<#assign dimModel = dimModel * dim>
</#list>
            self.${element.name} = DotProductSelfAttention(scale_factor=${scaleFactor}, num_heads=${numHeads}, dim_model=${dimModel}, dim_keys=${dimKeys}, dim_values=${dimValues}, use_proj_bias=${useProjBias})
			<#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${inputQueries}, ${inputKeys}, ${inputValues})
</#if>