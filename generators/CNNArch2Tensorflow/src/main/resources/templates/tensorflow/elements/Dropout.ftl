<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = tf.keras.layers.Dropout(rate = ${element.p?c}, name="${element.name}")(${element.inputs[0]})

