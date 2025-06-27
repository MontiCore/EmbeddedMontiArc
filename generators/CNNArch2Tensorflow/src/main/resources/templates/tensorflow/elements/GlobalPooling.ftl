<#-- (c) https://github.com/MontiCore/monticore -->
<#if element.poolType == "max">
        ${element.name} = tf.keras.layers.GlobalAvgPool2D(data_format = "channels_last", name="${element.name}")(${element.inputs[0]})
<#elseif element.poolType == "avg">
        ${element.name} = tf.keras.layers.GlobalMaxPool2D(data_format = "channels_last", name="${element.name}")(${element.inputs[0]})
<#else>
        print("Only max or avg pooling allowed for tensorflow (currently only 2D)") #make this an exception
</#if>
<#include "OutputShape.ftl">
