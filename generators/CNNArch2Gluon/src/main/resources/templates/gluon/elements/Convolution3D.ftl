<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.nn.Conv3D(channels=${element.channels?c},
                kernel_size=(${tc.join(element.kernel, ",")}),
                strides=(${tc.join(element.stride, ",")}),
                use_bias=${element.noBias?string("False", "True")},
                padding=(${tc.join(element.padding, ",")}))
<#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
