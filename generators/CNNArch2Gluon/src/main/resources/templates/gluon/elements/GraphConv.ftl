<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = GraphConv(in_feats=${element.inputDim},
                out_feats=${element.outputDim},
                bias=${element.noBias?string("False", "True")})
    <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ", ")})
</#if>
