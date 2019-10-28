<#-- This template is not used if the following architecture element is an output. See Output.ftl -->
<#assign axis = element.axis?c>
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Softmax(axis=${axis})
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
