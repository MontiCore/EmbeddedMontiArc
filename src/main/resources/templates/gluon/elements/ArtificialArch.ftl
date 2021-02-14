<#assign input = element.inputs[0]>
<#assign element = element.element>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = artificial_layer_${element.name}(<#list element.arguments as arg>${arg}=${arg} </#list>)
            # ${element.name}, output shape: {<#list element.outputTypes as type>[${tc.join(type.dimensions, ",")}]</#list>}
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
