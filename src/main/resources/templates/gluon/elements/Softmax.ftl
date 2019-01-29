<#-- This template is not used if the followiing architecture element is an output. See Output.ftl -->
<#assign input = element.inputs[0]>
<#assign mode = definition_mode.toString()>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Softmax()
</#if>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
