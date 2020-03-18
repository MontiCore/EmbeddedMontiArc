<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign subKeySize = element.subKeySize?c>
<#assign querySize = element.querySize?c>
<#assign actQuery = element.actQuery>
<#assign k = element.k?c>
<#assign numHeads = element.numHeads?c>
	
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Memory(sub_key_size=${subKeySize}, query_size=${querySize}, act_query="${actQuery}", k=${k}, num_heads=${numHeads})
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
