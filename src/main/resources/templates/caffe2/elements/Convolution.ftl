<#assign input = element.inputs[0]>
<#if element.padding??>
<#assign input = element.name>
        ${element.name} = mx.symbol.pad(data=${element.inputs[0]},  #TODO: pending to adapt
            mode='constant',
            pad_width=(${tc.join(element.padding, ",")}),
            constant_value=0)
</#if>
        ${element.name} = brew.conv(model, ${input}, '${element.name}', dim_in=1, dim_out=20, kernel=5)
<#include "OutputShape.ftl">
