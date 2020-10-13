<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef __${viewModel.callSolverName?upper_case}_H__
#define __${viewModel.callSolverName?upper_case}_H__
#include<armadillo>
#include<cppad/ipopt/solve.hpp>
#include "ADMat.h"

using CppAD::AD;
using namespace arma;

typedef CPPAD_TESTVECTOR(double) Dvector;
typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

namespace AnonymNS${viewModel.id}
{
  using CppAD::AD;
  using namespace arma;

  static Dvector gl = Dvector();
  static Dvector gu = Dvector();
  static ADvector g = ADvector();

  <#list viewModel.knownVariablesWithType as var>
  static ${var};
  </#list>

  class FG_eval_${viewModel.callSolverName} {
    public:
      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

      void operator()(ADvector &fg,const ADvector &x) {

        // create active optimization var
        <#if viewModel.optimizationVariableDimensions?size == 0>
        ${viewModel.optimizationVariableTypeActive} ${viewModel.optimizationVariableName} = x[0];
        <#else>
        ${viewModel.optimizationVariableTypeActive} ${viewModel.optimizationVariableName} = ${viewModel.optimizationVariableTypeActive}(<#list viewModel.optimizationVariableDimensions as dim>${dim}<#sep>, </#list>);
        for (int i${viewModel.id} = 0; i${viewModel.id} < x.size(); i${viewModel.id}++)
        {
          ${viewModel.optimizationVariableName}(i${viewModel.id}) = x[i${viewModel.id}];
        }
        </#if>

        // f(x)
        int i${viewModel.id} = 0;
        <#if viewModel.optimizationProblemType.name() == "MINIMIZATION">
        fg[i${viewModel.id}] = toADouble(${viewModel.objectiveFunction});
        <#else>
        fg[i${viewModel.id}] = -1 * toADouble( ${viewModel.objectiveFunction} );
        </#if>

        // g_i(x)
        i${viewModel.id}++;
        <#list viewModel.constraintFunctions as g>
        addConstraintFunction(${g}, fg, i${viewModel.id});
        </#list>
        //
        return;
      }

    private:
      adouble toADouble(const ADMat &value) { return value.at(0);}

      adouble toADouble(const adouble &value) { return value;}

      void addConstraintFunction(const adouble &value, ADvector &fg, int &i) {
        fg[i] = value;
        i++;
      }

      void addConstraintFunction(const ADMat &value, ADvector &fg, int &i) {
        for(int j = 0; j < value.size(); j++) {
          fg[i] = value[j];
          i++;
        }
      }
  };
}

using namespace arma;

class ${viewModel.callSolverName}
{
  private:

    static void addConstraint(const double &lower, const adouble &expr, const double &upper) {
      AnonymNS${viewModel.id}::gl.push_back(lower);
      AnonymNS${viewModel.id}::gu.push_back(upper);
      AnonymNS${viewModel.id}::g.push_back(expr);
    };

    static void addConstraint(const double &lower, const ADMat &expr, const double &upper) {
      for (int i = 0; i < expr.size(); i++) {
        AnonymNS${viewModel.id}::gl.push_back(lower);
        AnonymNS${viewModel.id}::gu.push_back(upper);
        AnonymNS${viewModel.id}::g.push_back(expr[i]);
      }
    };

    static void addConstraint(const double &lower, const ADMat &expr, const mat &upper) {
      assert(expr.size() == upper.size());
      for (int i = 0; i < expr.size(); i++) {
        AnonymNS${viewModel.id}::gl.push_back(lower);
        AnonymNS${viewModel.id}::gu.push_back(upper[i]);
        AnonymNS${viewModel.id}::g.push_back(expr[i]);
      }
    };

    static void addConstraint(const mat &lower, const ADMat &expr, const double &upper) {
      assert(lower.size() == expr.size());
      for (int i = 0; i < expr.size(); i++) {
        AnonymNS${viewModel.id}::gl.push_back(lower[i]);
        AnonymNS${viewModel.id}::gu.push_back(upper);
        AnonymNS${viewModel.id}::g.push_back(expr[i]);
      }
    };

    static void addConstraint(const mat &lower, const ADMat &expr, const mat &upper) {
      assert(lower.size() == expr.size());
      assert(expr.size() == upper.size());
      for (int i = 0; i < expr.size(); i++) {
        AnonymNS${viewModel.id}::gl.push_back(lower[i]);
        AnonymNS${viewModel.id}::gu.push_back(upper[i]);
        AnonymNS${viewModel.id}::g.push_back(expr[i]);
      }
    };

    static void addConstraint(const double &lower, const arma::subview_field<adouble> &expr, const double &upper) {
      for (int i = 0; i < expr.n_rows; i++) {
        for (int j = 0; j < expr.n_cols; j++) {
          AnonymNS${viewModel.id}::gl.push_back(lower);
          AnonymNS${viewModel.id}::gu.push_back(upper);
          AnonymNS${viewModel.id}::g.push_back(expr[i, j]);
        }
      }
    };

    static void addConstraint(const mat &lower, const arma::subview_field<adouble> &expr, const double &upper) {
      assert(lower.n_cols == expr.n_cols);
      assert(lower.n_rows == expr.n_rows);
      for (int i = 0; i < expr.n_rows; i++) {
        for (int j = 0; j < expr.n_cols; j++) {
          AnonymNS${viewModel.id}::gl.push_back(lower[i, j]);
          AnonymNS${viewModel.id}::gu.push_back(upper);
          AnonymNS${viewModel.id}::g.push_back(expr[i, j]);
        }
      }
    };

    static void addConstraint(const double &lower, const arma::subview_field<adouble> &expr, const mat &upper) {
      for (int i = 0; i < expr.n_rows; i++) {
        for (int j = 0; j < expr.n_cols; j++) {
          AnonymNS${viewModel.id}::gl.push_back(lower);
          AnonymNS${viewModel.id}::gu.push_back(upper[i, j]);
          AnonymNS${viewModel.id}::g.push_back(expr[i, j]);
        }
      }
    };

    static void addConstraint(const mat &lower, const arma::subview_field<adouble> &expr, const mat &upper) {
      for (int i = 0; i < expr.n_rows; i++) {
        for (int j = 0; j < expr.n_cols; j++) {
          AnonymNS${viewModel.id}::gl.push_back(lower[i, j]);
          AnonymNS${viewModel.id}::gu.push_back(upper[i, j]);
          AnonymNS${viewModel.id}::g.push_back(expr[i, j]);
        }
      }
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const double &lower, const int index, const double &upper) {
      xl[index] = std::fmax(xl[index], lower);
      xu[index] = std::fmin(xu[index], upper);
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const double &lower, const std::string access, const int index, const double &upper) {
      int dims[] = {<#list viewModel.optimizationVariableDimensions as dim>${dim?c}<#sep>, </#list>};
      int cols;
      int rows = dims[0];
      if (sizeof(dims) <= 1)
        cols = 1;
      else
        cols = dims[1];
      if (access.compare("col") == 0) {
        for (int i = index; i < xl.size(); i += rows) {
          xl[i] = std::fmax(xl[i], lower);
          xu[i] = std::fmin(xu[i], upper);
        }
      } else if (access.compare("row") == 0) {
          for (int i = index * cols; i < ((index + 1) * cols); i++) {
            xl[i] = std::fmax(xl[i], lower);
            xu[i] = std::fmin(xu[i], upper);
          }
      }
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const colvec &lower, const std::string access, const int index, const double &upper) {
      int dims[] = {<#list viewModel.optimizationVariableDimensions as dim>${dim?c}<#sep>, </#list>};
      int cols;
      int rows = dims[0];
      if (sizeof(dims) <= 1)
        cols = 1;
      else
        cols = dims[1];
      for (int i = index; i < xl.size(); i += rows) {
        xl[i] = std::fmax(xl[i], lower[i]);
        xu[i] = std::fmin(xu[i], upper);
      }
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const double &lower, const std::string access, const int index, const colvec &upper) {
      int dims[] = {<#list viewModel.optimizationVariableDimensions as dim>${dim?c}<#sep>, </#list>};
      int cols;
      int rows = dims[0];
      if (sizeof(dims) <= 1)
        cols = 1;
      else
        cols = dims[1];
      for (int i = index; i < xl.size(); i += rows) {
        xl[i] = std::fmax(xl[i], lower);
        xu[i] = std::fmin(xu[i], upper[i]);
      }
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const colvec &lower, const std::string access, const int index, const colvec &upper) {
      int dims[] = {<#list viewModel.optimizationVariableDimensions as dim>${dim?c}<#sep>, </#list>};
      int cols;
      int rows = dims[0];
      if (sizeof(dims) <= 1)
        cols = 1;
      else
        cols = dims[1];
      for (int i = index; i < xl.size(); i += rows) {
        xl[i] = std::fmax(xl[i], lower[i]);
        xu[i] = std::fmin(xu[i], upper[i]);
      }
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const rowvec &lower, const std::string access, const int index, const double &upper) {
      int dims[] = {<#list viewModel.optimizationVariableDimensions as dim>${dim?c}<#sep>, </#list>};
      int cols;
      int rows = dims[0];
      if (sizeof(dims) <= 1)
        cols = 1;
      else
        cols = dims[1];
      for (int i = index * cols; i < ((index + 1) * cols); i++) {
        xl[i] = std::fmax(xl[i], lower[i]);
        xu[i] = std::fmin(xu[i], upper);
      }
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const double &lower, const std::string access, const int index, const rowvec &upper) {
      int dims[] = {<#list viewModel.optimizationVariableDimensions as dim>${dim?c}<#sep>, </#list>};
      int cols;
      int rows = dims[0];
      if (sizeof(dims) <= 1)
        cols = 1;
      else
        cols = dims[1];
      for (int i = index * cols; i < ((index + 1) * cols); i++) {
        xl[i] = std::fmax(xl[i], lower);
        xu[i] = std::fmin(xu[i], upper[i]);
      }
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const rowvec &lower, const std::string access, const int index, const rowvec &upper) {
      int dims[] = {<#list viewModel.optimizationVariableDimensions as dim>${dim?c}<#sep>, </#list>};
      int cols;
      int rows = dims[0];
      if (sizeof(dims) <= 1)
        cols = 1;
      else
        cols = dims[1];
      for (int i = index * cols; i < ((index + 1) * cols); i++) {
        xl[i] = std::fmax(xl[i], lower[i]);
        xu[i] = std::fmin(xu[i], upper[i]);
      }
    };

    static void addConstraintOnX(Dvector &xl, Dvector &xu, const double &lower, const std::string access, const double &upper) {
      for (int i = 0; i < xl.size(); i++) {
        xl[i] = std::fmax(xl[i], lower);
        xu[i] = std::fmin(xu[i], upper);
      }
    };
    
  public:
    static bool solveOptimizationProblemIpOpt(
    ${viewModel.optimizationVariableType} &x${viewModel.id},
    double &y${viewModel.id}<#if 0 < viewModel.knownVariablesWithType?size>,</#if>
    <#list viewModel.knownVariablesWithType as arg>
    const ${arg}<#sep>,</#sep>
    </#list>
    )
    {
      bool ok = true;

      // declare opt var
      <#if viewModel.optimizationVariableDimensions?size == 0>
      ${viewModel.optimizationVariableTypeActive} ${viewModel.optimizationVariableName} = 0;
      <#else>
      ${viewModel.optimizationVariableTypeActive} ${viewModel.optimizationVariableName} = ${viewModel.optimizationVariableTypeActive}(<#list viewModel.optimizationVariableDimensions as dim>${dim}<#sep>, </#list>);
      </#if>

      // assign parameter variables
      <#list viewModel.knownVariables as var>
      AnonymNS${viewModel.id}::${var} = ${var};
      </#list>

      typedef CPPAD_TESTVECTOR(double)Dvector;

      // number of independent variables (domain dimension for f and g)
      size_t nx = ${viewModel.numberVariables?c};
      // number of constraints (range dimension for g)
      size_t ng = ${viewModel.numberConstraints?c};
      // initial value of the independent variables
      Dvector xi(nx);
      int i${viewModel.id} = 0;
      <#list viewModel.initX as x>
      xi[i${viewModel.id}] = ${x};
      i${viewModel.id}++;
      </#list>
      // lower and upper limits for x
      Dvector xl(nx),xu(nx);
      i${viewModel.id} = 0;
      <#list viewModel.xL as x>
      xl[i${viewModel.id}] = ${x};
      i${viewModel.id}++;
      </#list>
      i${viewModel.id} = 0;
      <#list viewModel.xU as x>
      xu[i${viewModel.id}] = ${x};
      i${viewModel.id}++;
      </#list>

      // limits for special matrix elements of x
      <#list viewModel.xMatrixElementConstraints as element>
      addConstraintOnX(xl, xu, ${element});
      </#list>

      // lower and upper limits for g
      Dvector gl(ng),gu(ng);
      <#list viewModel.constraintFunctions as g>
      addConstraint(${viewModel.gL[g?index]}, ${g}, ${viewModel.gU[g?index]});
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
        options, xi, xl, xu, AnonymNS${viewModel.id}::gl, AnonymNS${viewModel.id}::gu, fg_eval, solution);

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
