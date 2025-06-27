<#-- (c) https://github.com/MontiCore/monticore -->
<#assign channelIndex = element.element.outputTypes[0].channelIndex + 1>
<#assign heightIndex = element.element.outputTypes[0].heightIndex + 1>
<#assign widthIndex = element.element.outputTypes[0].widthIndex + 1>


<#assign indexList = []>
<#if heightIndex != 0><#assign indexList = indexList + [heightIndex]></#if>
<#if widthIndex != 0><#assign indexList = indexList + [widthIndex]></#if>
<#if channelIndex != 0><#assign indexList = indexList + [channelIndex]></#if>
        
<#if indexList?size != 3>
        #${element.name} = tf.keras.layers.Reshape(shape=(0,${element.element.outputTypes[0].channels?c},${element.element.outputTypes[0].height?c},${element.element.outputTypes[0].width?c}))(${element.name})
        ${element.name} = tf.keras.layers.Reshape(shape=(${element.element.outputTypes[0].channels?c},${element.element.outputTypes[0].height?c},${element.element.outputTypes[0].width?c}))(${element.name})
</#if>
               
        ${element.name} = tf.keras.layers.Input(shape=(${tc.join(element.element.outputTypes[0].dimensions, ",")}), name="${element.name}")
        input_tensors.append(${element.name})      
        
<#if widthIndex != heightIndex + 1 || channelIndex != widthIndex + 1>
        # We Want channels last for tensorflow
        # "tf_hwc_permute" name is used to check loaded networks for already existed permutation level, see LoadNetwork.ftl
        ${element.name} = tf.keras.layers.Permute((2,3,1), name="tf_hwc_permute")(${element.name})
        <#assign dimensions = element.element.outputTypes[0].dimensions[1..] + [element.element.outputTypes[0].dimensions[0]]>
</#if>        

<#include "OutputShape.ftl">               
            

        if not data_mean is None:
            assert(not data_std is None)

            ${element.name}  = tf.keras.layers.Lambda(lambda x : (x - data_mean["${element.name?keep_before_last("_")}"])/data_std["${element.name?keep_before_last("_")}"])(${element.name})
            ${element.name} = tf.keras.layers.Lambda(lambda x: tf.keras.backend.stop_gradient(x))(${element.name})
	
