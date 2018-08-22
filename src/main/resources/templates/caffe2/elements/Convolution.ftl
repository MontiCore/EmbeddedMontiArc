<#assign input = element.inputs[0]>
<#if element.padding??>  <#-- Check wheather padding null is. -->
<#assign input = element.name>
        #TODO: check how to adapt CNNArchLang argument pad_width=${element.padding}
</#if>
        ${element.name} = brew.conv(model, ${input}, '${element.name}', dim_in=1, dim_out=${element.channels?c}, kernel=${element.kernel}, stride=${element.stride})
        #TODO: check how to adapt CNNArchLang argument no_bias=${element.noBias?string("True","False")}
        #TODO: check how to adapt CNNArchLang argument pad_width=${element.padding}
<#include "OutputShape.ftl">
