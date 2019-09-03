<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
<#if element.padding??>
            self.${element.name}padding = Padding(padding=(${tc.join(element.padding, ",")}))
</#if>
<#if element.partOfUnroll>
            self.${element.name} = gluon.nn.Conv2D(channels=${element.channels?c},
                kernel_size=(${tc.join(element.kernel, ",")}),
                strides=(${tc.join(element.stride, ",")}),
                use_bias=${element.noBias?string("False", "True")},
                params=Net_${element.unrollIndex + tc.architecture.streams?size}().${element.name}.collect_params())
<#else>
            self.${element.name} = gluon.nn.Conv2D(channels=${element.channels?c},
                kernel_size=(${tc.join(element.kernel, ",")}),
                strides=(${tc.join(element.stride, ",")}),
                use_bias=${element.noBias?string("False", "True")})
</#if>
<#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
<#if element.padding??>
        ${element.name}padding = self.${element.name}padding(${input})
<#assign input = element.name + "padding">
</#if>
        ${element.name} = self.${element.name}(${input})
</#if>