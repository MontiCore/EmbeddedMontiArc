<#assign input = element.inputs[0]>
<#assign mode = definition_mode.toString()>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.nn.Activation(activation='relu')
</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>