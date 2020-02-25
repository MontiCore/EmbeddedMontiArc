<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign numSubKeys = element.numSubKeys?c>
<#assign querrySize = element.querrySize?c>
<#assign k = element.k?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = LargeMemory(numSubKeys=${numSubKeys}, querrySize=${querrySize}, k=${k})
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
