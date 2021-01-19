<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef __${viewModel.name?upper_case}_H__
#define __${viewModel.name?upper_case}_H__

#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "${viewModel.name}_RHS.h"
#include "ExecutionStepper.h"

typedef std::vector< double > state_type;
typedef boost::numeric::odeint::runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef boost::numeric::odeint::controlled_runge_kutta< error_stepper_type > controlled_stepper_type;

class ${viewModel.name} {
private:
    double lastTime = 0;
    double dt_max = ${viewModel.dt_max?string["0.0#######E0"]};
    state_type x;
    de_monticore_lang_monticar_semantics_loops_oscillation_EquationSystem_1_RHS rhs;
    controlled_stepper_type controlled_stepper;

    static controlled_stepper_type initStepper() {
    controlled_stepper_type res(
    boost::numeric::odeint::default_error_checker<
        double ,
        boost::numeric::odeint::range_algebra ,
        boost::numeric::odeint::default_operations>
        (${viewModel.atol?string["0.0#######E0"]}, ${viewModel.rtol?string["0.0#######E0"]}, 1.0, 1.0));
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
        rhs {},
        controlled_stepper {initStepper()} {}

    void execute() {
        <#list viewModel.inports as inport>
        rhs.${inport} = ${inport};
        </#list>
        <#if viewModel.inports?size gt 0>

        </#if>
        double t = getCurrentTime();
        boost::numeric::odeint::integrate_adaptive(
            controlled_stepper , rhs, x, lastTime, t, dt_max
        );

        lastTime = t;
        <#list viewModel.variables as var>
        ${var} = x[${var?index}];
        </#list>
    }
};
#endif