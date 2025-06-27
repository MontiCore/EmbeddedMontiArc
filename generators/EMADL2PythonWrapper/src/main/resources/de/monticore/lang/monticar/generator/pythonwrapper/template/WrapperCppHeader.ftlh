/* (c) https://github.com/MontiCore/monticore */
<#assign hasInputVariables = (inputVariables?size > 0)>
<#assign hasOutputVariables = (outputVariables?size > 0)>
#ifndef _${capitalizedWrapperClassName}_H_
#define _${capitalizedWrapperClassName}_H_
#include "armadillo"
#include "../${wrappedComponentName}.h"
<#if hasInputVariables>

struct ${inputClassName} {
<#list inputVariables as input>
    ${input.type} ${input.variableName};
</#list>
};
</#if>
<#if hasOutputVariables>

struct ${outputClassName} {
<#list outputVariables as output>
    ${output.type} ${output.variableName};
</#list>
};
</#if>

class ${wrapperClassName} {
private:
    ${wrappedComponentName} instance;
public:
    void init();
    <#if hasOutputVariables>${outputClassName}<#else>void</#if> execute(<#if hasInputVariables>${inputClassName} input</#if>);
};
#endif
