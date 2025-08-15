<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign subKeySize = element.subKeySize?c>
<#assign querySize = "[" + tc.join(element.querySize, ",") + "]">
<#assign queryAct = element.queryAct>
<#assign k = element.k?c>
<#assign numHeads = element.numHeads?c>
<#assign valuesDim = element.valuesDim?c>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = LargeMemory(sub_key_size=${subKeySize}, query_size=${querySize}, query_act="${queryAct}", 
                                               k=${k}, num_heads=${numHeads}, values_dim=${valuesDim})
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>