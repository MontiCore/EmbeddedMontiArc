<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = tf.keras.layers.Activation(activation = "relu", name="${element.name}")(${element.inputs[0]})

