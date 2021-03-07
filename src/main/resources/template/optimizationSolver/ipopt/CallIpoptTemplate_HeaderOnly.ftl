<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef __${viewModel.callSolverName?upper_case}_H__
#define __${viewModel.callSolverName?upper_case}_H__
#include"armadillo"
#include<cppad/ipopt/solve.hpp>
#include "ADMat.h"

using CppAD::AD;
using namespace arma;

typedef CPPAD_TESTVECTOR(double) Dvector;
typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

//Numbers of variables
#define V_N_ALLVARS ${viewModel.getNumberVariables()}
#define V_N_OPTVARS ${viewModel.getNumberOptimizationVariables()}
#define V_N_INDVARS ${viewModel.getNumberIndependentVariables()}

<#if viewModel.hasStepSize()>
// Step variable: ${viewModel.getStepSizeName()}
#define V_N_STEP_MAX ${viewModel.getStepSizeMax()?c}
#define V_N_STEP_MIN ${viewModel.getStepSizeMin()?c}
</#if>

//Offsets for ipopt vector
#define OPT_OFFSET 1
#define IND_OFFSET OPT_OFFSET + V_N_OPTVARS
#define CONSTR_OFFSET IND_OFFSET + V_N_INDVARS

//Optimization variables
<#list viewModel.optimizationVariables as optVar>
#define ${viewModel.getIpoptVarRef(optVar)} ${viewModel.getIpoptVarOffset(optVar)}
</#list>
//Independent variables
<#list viewModel.independentVariables as indVar>
#define ${viewModel.getIpoptVarRef(indVar)} ${viewModel.getIpoptVarOffset(indVar)}
</#list>
//Constraints
<#list viewModel.getConstraintFunctions() as constr>
#define V_CONSTRAINT_${constr?index?c} CONSTR_OFFSET + ${constr?index?c}
</#list>

namespace AnonymNS${viewModel.id}
{
  using CppAD::AD;
  using namespace arma;



  class FG_eval_${viewModel.callSolverName} {
    public:
      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

      void operator()(ADvector &fg,const ADvector &vars) {
        //fg[0] is evaluation function
        fg[0] = ${viewModel.getObjectiveFunction().getTextualRepresentation()};

        //Following fg entries are reserved for variables.
        //We use the defines to get a meaningful address space

        //Initialization
        <#list viewModel.optimizationVariables as var>
          fg[ 1 + ${viewModel.getIpoptVarRef(var)} ]  = ${viewModel.getVariableInitialization(var)};
        </#list>
        <#list viewModel.independentVariables as var>
          fg[ 1 + ${viewModel.getIpoptVarRef(var)} ]  = ${viewModel.getVariableInitialization(var)};
        </#list>

        //Updates
        <#if viewModel.hasStepSize()>
        for(int ${viewModel.getStepSizeName()} = ${viewModel.getStepSizeMin()?c}; ${viewModel.getStepSizeName()} < ${viewModel.getStepSizeMax()?c}; ${viewModel.getStepSizeName()}++)
        </#if>
        {
        //Constraint Functions (e.g. x(n+1) == x(n)*2)
        <#list viewModel.getConstraintFunctions() as constr>
            ${viewModel.getConstraintForFG_Eval(constr)};
        </#list>

        //Debug: Simplified Constraint Functions
                <#list viewModel.getSimplifiedConstraintFunctions() as constr>
                    ${viewModel.getConstraintForFG_Eval(constr)};
                </#list>
        }
        return;
      }

    private:
      adouble toADouble(const ADMat &value) { return value.at(0);}

      adouble toADouble(const adouble &value) { return value;}

  };
}

using namespace arma;

class ${viewModel.callSolverName}
{
  private:

    
  public:
    //ToDo: Function header generation, add initialization parameters
    static bool solveOptimizationProblemIpOpt()
    {
      bool ok = true;

      size_t n_vars = ${viewModel.getStepSizeCount()} * ${viewModel.getNumberIndependentVariables()} + (${viewModel.getStepSizeCount()}  - 1) * ${viewModel.getNumberOptimizationVariables()};
      size_t n_constraints = ${viewModel.getStepSizeCount()} * ${viewModel.getNumberConstraints()};

      Dvector vars_initial = Dvector(n_vars);
      Dvector vars_lowerbounds = Dvector(n_vars);
      Dvector vars_upperbounds = Dvector(n_vars);

      Dvector constraint_lowerbounds = Dvector(n_constraints);
      Dvector constraint_upperbounds = Dvector(n_constraints);

      // Initialize to zero / min / max, update actual initial values later
      for(int i=0;i<n_vars;i++){
        vars_initial[i] = 0;
        vars_lowerbounds[i] = -1.0E19;
        vars_upperbounds[i] =  1.0E19;
      }
      for(int i=0;i<n_constraints;i++){
        constraint_lowerbounds[i] = 0;
        constraint_upperbounds[i] = 0;
      }


      // Initialize variable bounds & initial value
      <#list viewModel.optimizationVariables as var>

        // Optimization variable: ${var.getName()}
        vars_initial[ ${viewModel.getIpoptVarRef(var)} ] = ${viewModel.getVariableInitialization(var)};
        <#if viewModel.hasStepSize()>
        for(int ${viewModel.getStepSizeName()} = ${viewModel.getStepSizeMin()?c}; ${viewModel.getStepSizeName()} < ${viewModel.getStepSizeMax()?c}; ${viewModel.getStepSizeName()}++){
            vars_lowerbounds[ ${viewModel.getIpoptVarRef(var)} + (${viewModel.getStepSizeName()}-${viewModel.getStepSizeMin()?c})] = ${viewModel.getVariableLowerBound(var)};
            vars_upperbounds[ ${viewModel.getIpoptVarRef(var)} + (${viewModel.getStepSizeName()}-${viewModel.getStepSizeMin()?c})] = ${viewModel.getVariableUpperBound(var)};
        }
        <#else>
        vars_lowerbounds[ ${viewModel.getIpoptVarRef(var)} ] = ${viewModel.getVariableLowerBound(var)};
        vars_upperbounds[ ${viewModel.getIpoptVarRef(var)} ] = ${viewModel.getVariableUpperBound(var)};
        </#if>
      </#list>

      <#list viewModel.independentVariables as var>
        // Independent variable: ${var.getName()}
        vars_initial[ ${viewModel.getIpoptVarRef(var)} ] = 0; //TBD
        <#if viewModel.hasStepSize()>
        for(int ${viewModel.getStepSizeName()} = ${viewModel.getStepSizeMin()?c}; ${viewModel.getStepSizeName()} < ${viewModel.getStepSizeMax()?c}; ${viewModel.getStepSizeName()}++){
            vars_lowerbounds[ ${viewModel.getIpoptVarRef(var)} + (${viewModel.getStepSizeName()}-${viewModel.getStepSizeMin()?c})] = ${viewModel.getVariableLowerBound(var)};
            vars_upperbounds[ ${viewModel.getIpoptVarRef(var)} + (${viewModel.getStepSizeName()}-${viewModel.getStepSizeMin()?c})] = ${viewModel.getVariableUpperBound(var)};
        }
        <#else>
        vars_lowerbounds[ ${viewModel.getIpoptVarRef(var)} ] = ${viewModel.getVariableLowerBound(var)};
        vars_upperbounds[ ${viewModel.getIpoptVarRef(var)} ] = ${viewModel.getVariableUpperBound(var)};
        </#if>
      </#list>

      // Initialize constraint bounds

      <#list viewModel.getConstraintFunctions() as constr>
        // Constraint: ${constr.getName()}

        <#if viewModel.hasStepSize()>
        for(int ${viewModel.getStepSizeName()} = ${viewModel.getStepSizeMin()?c}; ${viewModel.getStepSizeName()} < ${viewModel.getStepSizeMax()?c}; ${viewModel.getStepSizeName()}++){
            constraint_lowerbounds[ ${constr?index?c} + (n-1) * V_N_STEP] = ${viewModel.getConstraintLowerBound(constr)};
            constraint_upperbounds[ {constr?index?c} + (n-1) * V_N_STEP] = ${viewModel.getConstraintUpperBound(constr)};
        }
        <#else>
        constraint_lowerbounds[ ${constr?index?c} ] = ${viewModel.getConstraintLowerBound(constr)};
        constraint_upperbounds[ ${constr?index?c} ] = ${viewModel.getConstraintUpperBound(constr)};
        </#if>
      </#list>



      // object that computes objective and constraints
      AnonymNS${viewModel.id}::FG_eval_${viewModel.callSolverName} fg_eval;

      // options
      std::string options;
      <#list viewModel.options as option>
      options+="${option}\n";
      </#list>
      // place to return solution
      CppAD::ipopt::solve_result<Dvector> solution;

      // solve the problem
      CppAD::ipopt::solve<Dvector, AnonymNS${viewModel.id}::FG_eval_${viewModel.callSolverName}>(
        options, vars_initial, vars_lowerbounds, vars_upperbounds, constraint_lowerbounds, constraint_upperbounds, fg_eval, solution);

      // Check some of the solution values
      ok&=solution.status==CppAD::ipopt::solve_result<Dvector>::success;

      // assign solution values
      <#if viewModel.optimizationVariableDimensions?size == 0>
      x${viewModel.id} = solution.x[0];
      <#else>
      for (int i${viewModel.id} = 0; i${viewModel.id} < solution.x.size(); i${viewModel.id}++)
      {
        x${viewModel.id}(i${viewModel.id}) = solution.x[i${viewModel.id}];
      }
      </#if>
      // objective value
      <#if viewModel.optimizationProblemType.name() == "MINIMIZATION">
      y${viewModel.id} = solution.obj_value;
      <#else>
      y${viewModel.id} = -1 * solution.obj_value;
      </#if>

      // print short message
      std::cout<<std::endl<<std::endl<<"Solving status: "<<solution.status<<"!"<<std::endl;
      std::cout<<"${viewModel.optimizationProblemType.name()?capitalize} variable value: "<<std::endl<<"x = "<<std::endl<<x${viewModel.id}<<std::endl;
      std::cout<<"${viewModel.optimizationProblemType.name()?capitalize} objective value: "<<std::endl<<"y = "<<y${viewModel.id}<<std::endl;
      return ok;
    }
};

#endif

/*
Debug:
${viewModel.listClassesInScope()}
*/