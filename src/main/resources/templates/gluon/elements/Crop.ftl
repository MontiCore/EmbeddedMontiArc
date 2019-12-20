<#-- (c) https://github.com/MontiCore/monticore -->
<#--- http://beta.mxnet.io/r/api/mx.symbol.slice.html -->
<#if element.inputs?size gte 1>
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.slice(start=(${tc.join(element.start, ",")}),end=(${tc.join(element.end, ",")}))
</#if>
</#if>
