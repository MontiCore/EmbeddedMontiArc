<#-- (c) https://github.com/MontiCore/monticore -->
<#-- This template is not used if the followiing architecture element is an output. See Output.ftl -->
        <#--  ${element.name} = mx.symbol.softmax(data=${element.inputs[0]},
            axis=1,
            name="${element.name}"
        )  -->

<#if element.axis == -1>
<#assign axis = element.axis?c>
<#else>
<#assign axis = (element.axis + 1)?c>
</#if>
<#assign input = element.inputs[0]>
        ${element.name} = mx.symbol.softmax(data=${element.inputs[0]},
            axis=${axis},
            name="${element.name}"
        )
