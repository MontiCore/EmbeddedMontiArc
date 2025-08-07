<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = tf.keras.layers.Lambda(lambda x: tf.nn.local_response_normalization(input=x,
                                                                                              depth_radius=${element.nsize?c} / 2, #instead element.depth_radius?
                                                                                              bias=${element.knorm?c}, #instead element.bias?												 
                                                                                              alpha=${element.alpha?c},
                                                                                              beta=${element.beta?c},
                                                                                              name="${element.name}"))(${element.inputs[0]})

