<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef __${viewModel.name?upper_case}_H__
#define __${viewModel.name?upper_case}_H__

#include "solver.h"
#include "${viewModel.name}_MassMatrix.h"
#include "${viewModel.name}_RHS.h"
#include "ExecutionStepper.h"

class ${viewModel.name} {
private:
    daecpp::state_type x;
    ${viewModel.name}_MassMatrix mass;
    ${viewModel.name}_RHS rhs;
    daecpp::Jacobian jac;
    daecpp::SolverOptions opt;
    daecpp::Solver solver;

    static daecpp::SolverOptions initOptions() {
        daecpp::SolverOptions res;
        <#if viewModel.isAlgebraic>
        res.time_stepping = 1;
        </#if>
        res.atol = ${viewModel.atol?string["0.0#######E0"]};
        res.dt_init = ${viewModel.dt_init?string["0.0#######E0"]};
        res.dt_max = ${viewModel.dt_max?string["0.0#######E0"]};
        res.verbosity = ${viewModel.loggingLevel};
        return res;
    }

public:
<#list viewModel.inports as inport>
    double ${inport};
</#list>

<#list viewModel.variables as var>
    double ${var};
</#list>

    ${viewModel.name}() :
        x {${viewModel.initialValues?join(", ")}},
        mass {},
        rhs {},
        jac {rhs, ${viewModel.jtol?string["0.0#######E0"]}},
        opt { initOptions() },
        solver {rhs, jac, mass, opt} {}

    void execute() {
        <#list viewModel.inports as inport>
        rhs.${inport} = ${inport};
        </#list>
        <#if viewModel.inports?size gt 0>

        </#if>
        double t = getCurrentTime();
        solver(x, t);
        <#list viewModel.variables as var>
        ${var} = x[${var?index}];
        </#list>
    }
};
#endif