<#assign input = element.inputs[0]>
<#if element.padding??>
<#assign input = element.name>
        ${element.name} = mx.symbol.pad(data=${element.inputs[0]},  #TODO: Pending to adapt o eliminate
            mode='constant',
            pad_width=(${tc.join(element.padding, ",")}),
            constant_value=0)
</#if>
        ${element.name} = brew.max_pool(model, ${input}, '${element.name}', kernel=2, stride=2)




