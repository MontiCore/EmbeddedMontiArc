<#-- (c) https://github.com/MontiCore/monticore -->
<#if element.inputs?size gte 1>
<#assign input = element.inputs[0]>

<#if mode == "ARCHITECTURE_DEFINITION">
        self.${element.name} = nn.Identity()
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})

        return ${element.name}
</#if>
</#if>