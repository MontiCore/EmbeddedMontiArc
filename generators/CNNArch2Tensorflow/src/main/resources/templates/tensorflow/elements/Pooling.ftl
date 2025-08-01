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
    <#if element.poolType == "max">
        ${element.name} = tf.keras.layers.MaxPool2D(pool_size = (${tc.join(element.kernel, ",")}), #or element.poolsize?
            strides = (${tc.join(element.stride, ",")}), #or element.strides?  (plural)
            padding="${pad}",            
            data_format = "channels_last",
            name="${element.name}")(${element.inputs[0]})
	<#elseif element.poolType == "avg">
        ${element.name} = tf.keras.layers.AvgPool2D(pool_size = (${tc.join(element.kernel, ",")}), #or element.poolsize?
            strides = (${tc.join(element.stride, ",")}), #or element.strides?  (plural)
            padding="${pad}",
            data_format = "channels_last",
            name="${element.name}")(${element.inputs[0]})
	<#else>
            print("Only max or avg pooling allowed for tensorflow (currently only 2D)") #make this an exception
	</#if>
<#include "OutputShape.ftl">
