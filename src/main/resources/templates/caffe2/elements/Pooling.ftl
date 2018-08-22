<#assign input = element.inputs[0]>
<#if element.padding??>
<#assign input = element.name>
        #TODO: check how to adapt CNNArchLang argument pad_width=${element.padding}
</#if>

<#if element.poolType == "max">
        ${element.name} = brew.max_pool(model, ${input}, '${element.name}', kernel=${element.kernel}, stride=${element.stride})
<#elseif element.poolType == "avg">
        ${element.name} = brew.average_pool(model, ${input}, '${element.name}', kernel=${element.kernel}, stride=${element.stride})
</#if>


<#include "OutputShape.ftl">