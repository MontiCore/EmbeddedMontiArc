<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if !element.knorm?string?contains(".")>
    <#assign bias = element.knorm?string["0.0"]>
<#else>
    <#assign bias = element.knorm?c>
</#if>
            ${element.name} = brew.lrn(model, ${input}, '${element.name}', size=${element.nsize?c}, alpha=${element.alpha?c}, beta=${element.beta?c}, bias=${bias})
