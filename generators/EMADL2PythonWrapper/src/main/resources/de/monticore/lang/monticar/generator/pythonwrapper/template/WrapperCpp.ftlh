/* (c) https://github.com/MontiCore/monticore */
<#assign hasInputVariables = (inputVariables?size > 0)>
<#assign hasOutputVariables = (outputVariables?size > 0)>
#include "${wrapperClassName}.h"

void ${wrapperClassName}::init() {
    instance.init();
}

<#if hasOutputVariables>${outputClassName}<#else>void</#if> ${wrapperClassName}::execute(<#if hasInputVariables>${inputClassName} input</#if>) {
<#if hasOutputVariables>
    ${outputClassName} output;

</#if>
<#list inputVariables as input>
    instance.${input.variableName} = input.${input.variableName};
</#list>

    instance.execute();

<#list outputVariables as output>
    output.${output.variableName} = instance.${output.variableName};
</#list>
<#if hasOutputVariables>
    return output;
</#if>
}
