<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>

<#if mode == "ARCHITECTURE_DEFINITION">
        self.${element.name} = nn.ReLU()
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>