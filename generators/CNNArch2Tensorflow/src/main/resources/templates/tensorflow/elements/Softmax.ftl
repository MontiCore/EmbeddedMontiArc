<#-- (c) https://github.com/MontiCore/monticore -->
<#-- This template is not used if the followiing architecture element is an output. See Output.ftl, actually currently it is used when using Sofmax() in the emadl file -->
        ${element.name} = tf.keras.layers.Softmax(name="${element.name}")(${element.inputs[0]})

