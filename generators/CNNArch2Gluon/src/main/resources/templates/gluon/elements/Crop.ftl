<#-- (c) https://github.com/MontiCore/monticore -->
<#--- http://beta.mxnet.io/r/api/mx.symbol.slice.html -->
<#if element.until == 0>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.slice(start=(None, None, ${element.start}, ${element.start}), end=(None, None, None, None))
</#if>
<#else>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.slice(start=(None, None, ${element.start}, ${element.start}), end=(None, None, ${element.until}, ${element.until}))
</#if>
</#if>