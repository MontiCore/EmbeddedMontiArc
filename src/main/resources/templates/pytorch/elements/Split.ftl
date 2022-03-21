<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign num_outputs = element.numOutputs?c>

<#if mode == "FORWARD_FUNCTION">
        ${element.name} = torch.tensor_split(${input}, ${num_outputs},dim=1)
<#include "OutputShape.ftl">
</#if>