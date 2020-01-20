<#-- (c) https://github.com/MontiCore/monticore -->
<#--- http://beta.mxnet.io/r/api/mx.symbol.slice.html -->
<#if element.inputs?size gte 1>
<#assign input = element.inputs[0]>
<#if mode == "FORWARD_FUNCTION">
        ${element.name}_start = (None, None, ${tc.join(element.start, ", ")})
        ${element.name}_end = (None, None, ${tc.join(element.endd, ", ")})
        ${element.name} = F.slice(start=${element.name}_start, end=${element.name}_end)
</#if>
</#if>
