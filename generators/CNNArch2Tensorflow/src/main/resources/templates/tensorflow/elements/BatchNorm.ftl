<#-- (c) https://github.com/MontiCore/monticore -->
        ${element.name} = tf.keras.layers.BatchNormalization(beta_regularizer=self._regularizer_,
                                                             gamma_regularizer=self._regularizer_,
                                                             beta_constraint=self._weight_constraint_,
                                                             gamma_constraint=self._weight_constraint_,
                                                             name="${element.name}",)(${element.inputs[0]}) #TODO: fix_gamma=${element.fixGamma?string("True","False")}

