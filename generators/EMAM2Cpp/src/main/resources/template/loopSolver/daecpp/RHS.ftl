<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef __${viewModel.name?upper_case}_RHS_H__
#define __${viewModel.name?upper_case}_RHS_H__

#include "solver.h"

class ${viewModel.name}_RHS : public daecpp::RHS {
public:
<#list viewModel.inports as inport>
    double ${inport};
</#list>
<#if viewModel.inports?size gt 0>

</#if>
    void operator()(const daecpp::state_type &x, daecpp::state_type &f,
                    const double t) {
        <#list viewModel.function as f>
        f[${f?index}] = ${f};
        </#list>
    }
};
#endif