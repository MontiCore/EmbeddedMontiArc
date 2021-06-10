<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = architecture_defined_block_${element.element.name}()
            #${element.name}, output shape: {<#list element.element.outputTypes as type>[${tc.join(type.dimensions, ",")}]</#list>}
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
