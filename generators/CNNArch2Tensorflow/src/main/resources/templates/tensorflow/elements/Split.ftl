<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = tf.keras.layers.Lambda(lambda x: tf.split(x,num_or_size_splits=${element.numOutputs?c},axis=3))(${element.inputs[0]})     
<#include "OutputShape.ftl">
