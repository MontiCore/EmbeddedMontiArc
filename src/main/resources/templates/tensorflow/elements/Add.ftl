        ${element.name} = tf.keras.layers.Add()([${tc.join(element.inputs, ",  ")}])
<#include "OutputShape.ftl">
