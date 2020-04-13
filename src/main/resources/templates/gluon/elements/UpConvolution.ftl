<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
<#if element.padding??>
            self.${element.name}padding = (${tc.join(element.transPadding, ",")})
</#if>
            self.${element.name} = gluon.nn.Conv2DTranspose(channels=${element.channels?c},
                kernel_size=(${tc.join(element.kernel, ",")}),
                strides=(${tc.join(element.stride, ",")}),
                padding=self.${element.name}padding,
                use_bias=${element.noBias?string("False", "True")})
<#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
