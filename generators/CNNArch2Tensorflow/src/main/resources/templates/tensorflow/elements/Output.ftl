<#-- (c) https://github.com/MontiCore/monticore -->
        #Just an "Identity" layer with the appropriate output name, as if softmax is required it is generated from the Softmax.ftl template        
        ${element.name} = tf.keras.layers.Lambda(lambda x: x, name="${element.name}")(${element.inputs[0]})
        output_names.append("${element.name}")
        
        <#assign ouput_dimensions = tc.architecture.outputs[0].ioDeclaration.type.dimensions>
<#if ouput_dimensions?size == 3>
        #When the output has 3 dimensions, it is assumed that it can be interpreted as having channels, as the generator pipeline works generaly with channels first format, 
        #but tensorflow with channels last we reverse the oredering in the end
        ${element.name} = tf.keras.layers.Permute((3,1,2))(${element.name})
</#if>

