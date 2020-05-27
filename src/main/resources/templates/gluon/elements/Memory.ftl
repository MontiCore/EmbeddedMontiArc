<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign subKeySize = element.subKeySize?c>
<#assign querySize = "[" + tc.join(element.querySize, ",") + "]">
<#assign queryAct = element.queryAct>
<#assign storeDistMeasure = element.storeDistMeasure>
<#assign k = element.k?c>
<#assign numHeads = element.numHeads?c>
<#assign valueShape = "(" + tc.join(element.valueShape, ",") + ",)">
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = Memory(sub_key_size=${subKeySize}, query_size=${querySize}, query_act="${queryAct}", 
                                          dist_measure="${storeDistMeasure}", k=${k}, num_heads=${numHeads}, 
                                          value_shape=${valueShape})
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>