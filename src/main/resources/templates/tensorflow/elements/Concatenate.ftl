        ${element.name} = tf.keras.layers.Concatenate(axis=-1, name="${element.name}")([${tc.join(element.inputs, ",  ")}])
<#include "OutputShape.ftl">
