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
        res.atol = 1.0e-15;
        res.dt_init = 0.01;
        res.dt_max = 0.01;
        res.verbosity = 0;
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
        jac {rhs, 1.0e-15},
        opt { initOptions() },
        solver {rhs, jac, mass, opt} {
    }

    void execute() {
        <#list viewModel.inports as inport>
        rhs.${inport} = ${inport};
        </#list>

        double t = getCurrentTime();
        solver(x, t);
<#list viewModel.variables as var>
        ${var} = x[${var?index}];
</#list>
    }
};
#endif