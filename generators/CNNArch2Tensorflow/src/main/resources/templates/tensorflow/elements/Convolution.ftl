<#-- (c) https://github.com/MontiCore/monticore -->
<#if element.padding??>
        <#assign paddingTupel = tc.join(element.padding, ",")>
        <#if paddingTupel == "0,-1,0,0,0,0,0,0">
                <#assign pad = "valid">
        <#elseif paddingTupel == "0,0,-1,0,0,0,0,0">
                <#assign pad = "no_loss">
        <#else>
                <#assign pad = "same">
        </#if>
<#else>
        <#assign pad = "same">
</#if>
        ${element.name} = tf.keras.layers.Conv2D(${element.channels?c}, 
                                                 kernel_size=(${tc.join(element.kernel, ",")}), 
                                                 strides=(${tc.join(element.stride, ",")}), 
                                                 use_bias=${element.noBias?string("False","True")},
                                                 padding="${pad}",
                                                 kernel_regularizer=self._regularizer_, 
                                                 kernel_constraint=self._weight_constraint_, 
                                                 name="${element.name}")(${element.inputs[0]})
<#include "OutputShape.ftl">
