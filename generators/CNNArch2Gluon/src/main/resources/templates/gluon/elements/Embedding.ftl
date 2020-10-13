<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.nn.Embedding(input_dim=${element.inputDim?c}, output_dim=${element.outputDim?c})
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
