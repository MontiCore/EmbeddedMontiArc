<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
        <#if element.partOfUnroll>
            self.${element.name} = gluon.nn.Embedding(input_dim=${element.inputDim?c}, output_dim=${element.outputDim?c},
                    params=Net_${element.unrollIndex + tc.architecture.streams?size}().${element.name}.collect_params())
        <#else>
            self.${element.name} = gluon.nn.Embedding(input_dim=${element.inputDim?c}, output_dim=${element.outputDim?c})
        </#if>
        <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>