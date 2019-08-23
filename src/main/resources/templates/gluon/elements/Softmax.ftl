<#-- (c) https://github.com/MontiCore/monticore -->
<#-- This template is not used if the followiing architecture element is an output. See Output.ftl -->
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Softmax()
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
