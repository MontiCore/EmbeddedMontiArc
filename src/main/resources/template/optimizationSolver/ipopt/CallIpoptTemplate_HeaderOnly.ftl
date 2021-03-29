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
<#list viewModel.getSimplifiedConstraintFunctions() as constr>
#define ${viewModel.getIpoptConstraintRef(constr?index)} ${viewModel.getIpoptConstraintOffset(constr?index)}
</#list>
/*
mat vec2mat(vec V, size_t cols) {
    size_t rows = std::ceil(V.n_elems / double(cols));
    return V.reshape(cols, rows);// return the original vector as matrix
}
*/



namespace AnonymNS${viewModel.id}
{
  using CppAD::AD;
  using namespace arma;

  //constant variables
  <#list viewModel.getExternalVariables() as ext>
  ${viewModel.getExternalVariableType(ext)} ${ext.getName()};
  </#list>

  class FG_eval_${viewModel.callSolverName} {
    public:
      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

      void operator()(ADvector &fg,const ADvector &vars) {

        <#if !viewModel.hasStepSize()>
        <#list viewModel.optimizationVariables as var>
        ADMat ${var.getName()} = ADMat(${viewModel.getVariableDimensionM(var)!c},${viewModel.getVariableDimensionN(var)!c});
        for(int m = 0; m < ${viewModel.getVariableDimensionM(var)!c};m++){
            for(int n = 0; n < ${viewModel.getVariableDimensionN(var)!c};n++){
                int i = m*${viewModel.getVariableDimensionM(var)!c} + n;
                ${var.getName()}(i) = vars[${viewModel.getIpoptVarRef(var)} + i];
            }
        }
        </#list>
        </#if>

        //fg[0] is evaluation function
        <#if !viewModel.hasStepSize()>
        fg[0] = ${viewModel.getRawObjectiveFunction()};
        <#else>
        fg[0] = ${viewModel.getObjectiveFunctionWithIpoptVectorEntries()};

        //Following fg entries are reserved for variables.
        //We use the defines to get a meaningful address space

        //Independent Variable Initialization
        <#list viewModel.optimizationVariables as var>
          fg[ 1 + ${viewModel.getIpoptVarRef(var)} ]  = ${viewModel.getVariableInitialization(var)};
        </#list>
        <#list viewModel.independentVariables as var>
          fg[ 1 + ${viewModel.getIpoptVarRef(var)} ]  = ${viewModel.getVariableInitialization(var)};
        </#list>
        </#if>

        //Updates
        <#if viewModel.hasStepSize()>
        for(int ${viewModel.getStepSizeName()} = ${viewModel.getStepSizeMin()?c}; ${viewModel.getStepSizeName()} < ${viewModel.getStepSizeMax()?c}; ${viewModel.getStepSizeName()}++)
        </#if>
        {
        // Constraint Functions
        <#list viewModel.getSimplifiedConstraintFunctions() as constr>
            ${viewModel.getConstraintForFG_Eval(constr,constr?index)};
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
    static bool solveOptimizationProblemIpOpt(${viewModel.getIpoptSolverFunctionParameters()})
    {
      bool ok = true;

      <#if !viewModel.hasStepSize()>
      size_t n_vars = ${viewModel.getOptVarDimension()} + ${viewModel.getIndVarDimension()};
      size_t n_constraints = ${viewModel.getSimplifiedConstraintFunctionCount()};
      <#else>
      size_t n_vars = ${viewModel.getStepSizeCount()} * ${viewModel.getNumberIndependentVariables()} + (${viewModel.getStepSizeCount()}  - 1) * ${viewModel.getNumberOptimizationVariables()};
      size_t n_constraints = ${viewModel.getStepSizeCount()} * ${viewModel.getNumberConstraints()};
      </#if>
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
        for(int m = 0; m < ${viewModel.getVariableDimensionM(var)!c};m++){
          for(int n = 0; n < ${viewModel.getVariableDimensionN(var)!c};n++){
             int i = m*${viewModel.getVariableDimensionM(var)!c} + n;
             vars_lowerbounds[ ${viewModel.getIpoptVarRef(var)} + i] = ${viewModel.getVariableLowerBound(var)};
             vars_upperbounds[ ${viewModel.getIpoptVarRef(var)} + i] = ${viewModel.getVariableUpperBound(var)};
          }
        }
        </#if>
      </#list>

      <#list viewModel.independentVariables as var>
        // Independent variable: ${var.getName()}
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

      // Initialize constraint bounds
      <#list viewModel.getSimplifiedConstraintFunctions() as constr>
        // Constraint: ${constr.getName()}

        <#if viewModel.hasStepSize()>
        for(int ${viewModel.getStepSizeName()} = ${viewModel.getStepSizeMin()?c}; ${viewModel.getStepSizeName()} < ${viewModel.getStepSizeMax()?c}; ${viewModel.getStepSizeName()}++){
            constraint_lowerbounds[ ${viewModel.getIpoptConstraintRef(constr?index)} + (${viewModel.getStepSizeName()}-${viewModel.getStepSizeMin()?c})] = ${viewModel.getConstraintLowerBound(constr)};
            constraint_upperbounds[ ${viewModel.getIpoptConstraintRef(constr?index)} + (${viewModel.getStepSizeName()}-${viewModel.getStepSizeMin()?c})] = ${viewModel.getConstraintUpperBound(constr)};
        }
        <#else>
        constraint_lowerbounds[ ${viewModel.getIpoptConstraintRef(constr?index)} ] = ${viewModel.getConstraintLowerBound(constr)};
        constraint_upperbounds[ ${viewModel.getIpoptConstraintRef(constr?index)} ] = ${viewModel.getConstraintUpperBound(constr)};
        </#if>
      </#list>

      //Push constants to namespace
      <#list viewModel.getExternalVariables() as ext>
        AnonymNS${viewModel.id}::${ext.getName()} = ${ext.getName()}; //${viewModel.getExternalVariableType(ext)}
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
      <#list viewModel.getOptimizationVariables() as opt>
      <#if viewModel.hasStepSize()>
        for(int ${viewModel.getStepSizeName()} = ${viewModel.getStepSizeMin()?c}; ${viewModel.getStepSizeName()} < ${viewModel.getStepSizeMax()?c}; ${viewModel.getStepSizeName()}++){
            ${opt.getName()}[${viewModel.getStepSizeName()}] = solution.x[${viewModel.getIpoptVarRef(opt)} + ${viewModel.getStepSizeName()}];
        }
      <#else>
        <#if viewModel.isVarScalar(opt)>
        ${opt.getName()} = solution.x[${viewModel.getIpoptVarRef(opt)}];
        <#else>
        for(int m = 0; m < ${viewModel.getVariableDimensionM(opt)!c};m++){
          for(int n = 0; n < ${viewModel.getVariableDimensionN(opt)!c};n++){
             int i = m*${viewModel.getVariableDimensionM(opt)!c} + n;
             ${opt.getName()}[i] = solution.x[${viewModel.getIpoptVarRef(opt)} + i];
          }
        }
        </#if>
      </#if>
      </#list>
      // objective value
      <#if viewModel.optimizationProblemType.name() == "MINIMIZATION">
      *${viewModel.getObjectiveVariableName()} = solution.obj_value;
      <#else>
      *${viewModel.getObjectiveVariableName()} = -1 * solution.obj_value;
      </#if>

      // print short message
      std::cout<<std::endl<<std::endl<<"Solving status: "<<solution.status<<"!"<<std::endl;
      return ok;
    }
};

#endif