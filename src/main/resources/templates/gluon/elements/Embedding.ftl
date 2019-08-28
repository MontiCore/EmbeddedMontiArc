<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
        <#if element.partOfUnroll>
            ${element.name} = Net_1.${element.name}(${input})
        <#else>
            self.${element.name} = gluon.nn.Embedding(input_dim=${element.inputDim?c}, output_dim=${element.outputDim?c})
        </#if>
        <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
    <#if element.partOfUnroll>
        ${element.name} = Net_1.${element.name}(${input})
    <#else>
        ${element.name} = self.${element.name}(${input})
    </#if>
</#if>