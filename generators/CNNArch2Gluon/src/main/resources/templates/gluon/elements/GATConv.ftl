<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = GATConv(in_feats=${element.inputDim}, out_feats=${element.outputDim}, num_heads=${element.numHeads})
    <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ", ")})
</#if>
