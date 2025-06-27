<#-- (c) https://github.com/MontiCore/monticore -->
<#assign flatten = element.element.inputTypes[0].height != 1 || element.element.inputTypes[0].width != 1>
<#assign input = element.inputs[0]>
<#if flatten>
        ${element.name} = tf.keras.layers.Flatten()(${input})
<#assign input = element.name>
</#if>

        ${element.name} = tf.keras.layers.Dense(${element.units?c},
                                                use_bias=${element.noBias?string("True","False")},
                                                kernel_regularizer=self._regularizer_, 
                                                kernel_constraint=self._weight_constraint_, 
                                                name="${element.name}")(${input})

