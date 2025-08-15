<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef __${viewModel.name?upper_case}_RHS_H__
#define __${viewModel.name?upper_case}_RHS_H__

#include <boost/numeric/odeint.hpp>

typedef std::vector< double > state_type;

class ${viewModel.name}_RHS {
public:
<#list viewModel.inports as inport>
    double ${inport};
</#list>
<#if viewModel.inports?size gt 0>

</#if>
    void operator() ( const state_type &x , state_type &dxdt , const double /* t */ )
    {
        <#list viewModel.function as f>
        dxdt[${f?index}] = ${f};
        </#list>
    }
};
#endif