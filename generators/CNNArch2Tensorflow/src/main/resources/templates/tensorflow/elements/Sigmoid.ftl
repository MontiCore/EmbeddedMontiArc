<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = tf.keras.layers.Activation("sigmoid", name="${element.name}")(${element.inputs[0]})
