<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef __${viewModel.callSolverName?upper_case}_H__
#define __${viewModel.callSolverName?upper_case}_H__

#include <ilcplex/ilocplex.h>
#include "armadillo"
#include "CplexMat.h"


class ${viewModel.callSolverName}
{
	private:

    static IloNumExprArg toScalar(const CplexMat &value) { return value.at(0);}

    static IloNumExprArg toScalar(const IloNumExprArg &value) { return value;}

    static void addConstraint(IloRangeArray &con${viewModel.id}, const double &lower, const IloNumExprArg &expr, const double &upper) {
      con${viewModel.id}.add(lower <= expr <= upper);
    };

    static void addConstraint(IloRangeArray &con${viewModel.id}, const double &lower, const CplexMat &expr, const double &upper) {
      for (int i = 0; i < expr.size(); i++) {
        con${viewModel.id}.add(lower <= expr[i] <= upper);
      }
    };

    static void addConstraint(IloRangeArray &con${viewModel.id}, const double &lower, const CplexMat &expr, const mat &upper) {
      assert(expr.size() == upper.size());
      for (int i = 0; i < expr.size(); i++) {
        con${viewModel.id}.add(lower <= expr[i] <= upper[i]);
      }
    };

    static void addConstraint(IloRangeArray &con${viewModel.id}, const mat &lower, const CplexMat &expr, const double &upper) {
      assert(lower.size() == expr.size());
      for (int i = 0; i < expr.size(); i++) {
        con${viewModel.id}.add(lower[i] <= expr[i] <= upper);
      }
    };

    static void addConstraint(IloRangeArray &con${viewModel.id}, const mat &lower, const CplexMat &expr, const mat &upper) {
      assert(lower.size() == expr.size());
      assert(expr.size() == upper.size());
      for (int i = 0; i < expr.size(); i++) {
        con${viewModel.id}.add(lower[i] <= expr[i] <= upper[i]);
      }
    };

    static void addConstraint(IloRangeArray &con${viewModel.id}, const double &lower, const arma::subview_field<IloNumExprArg> &expr, const double &upper) {
      for (int i = 0; i < expr.n_rows; i++) {
        for (int j = 0; j < expr.n_cols; j++) {
          con${viewModel.id}.add(lower <= expr[i, j] <= upper);
        }
      }
    };

    static void addConstraint(IloRangeArray &con${viewModel.id}, const mat &lower, const arma::subview_field<IloNumExprArg> &expr, const double &upper) {
      assert(lower.n_cols == expr.n_cols);
      assert(lower.n_rows == expr.n_rows);
      for (int i = 0; i < expr.n_rows; i++) {
        for (int j = 0; j < expr.n_cols; j++) {
          con${viewModel.id}.add(lower[i, j] <= expr[i, j] <= upper);
        }
      }
    };

    static void addConstraint(IloRangeArray &con${viewModel.id}, const double &lower, const arma::subview_field<IloNumExprArg> &expr, const mat &upper) {
      for (int i = 0; i < expr.n_rows; i++) {
        for (int j = 0; j < expr.n_cols; j++) {
          con${viewModel.id}.add(lower <= expr[i, j] <= upper[i, j]);
        }
      }
    };

    static void addConstraint(IloRangeArray &con${viewModel.id}, const mat &lower, const arma::subview_field<IloNumExprArg> &expr, const mat &upper) {
      for (int i = 0; i < expr.n_rows; i++) {
        for (int j = 0; j < expr.n_cols; j++) {
          con${viewModel.id}.add(lower[i, j] <= expr[i, j] <= upper[i, j]);
        }
      }
    };

	public:
    static bool solveOptimizationProblemCplex(
      ${viewModel.optimizationVariableType} &x${viewModel.id},
      double &y${viewModel.id}<#if 0 < viewModel.knownVariablesWithType?size>,</#if>
      <#list viewModel.knownVariablesWithType as arg>
      const ${arg}<#sep>,</#sep>
      </#list>
    )
    {
      ILOSTLBEGIN
      IloEnv env;
      	try {
      		IloModel model(env);
      		IloNumVarArray var${viewModel.id}(env);
      		IloRangeArray con${viewModel.id}(env);
          IloEnv env = model.getEnv();
          <#list viewModel.xL as xl>
          var${viewModel.id}.add(IloNumVar(env, ${xl}, ${viewModel.xU[xl?index]}));
          </#list>
          // create active optimization var
          <#if viewModel.optimizationVariableDimensions?size == 0>
          ${viewModel.optimizationVariableTypeActive} ${viewModel.optimizationVariableName} = var${viewModel.id}[0];
          <#else>
          ${viewModel.optimizationVariableTypeActive} ${viewModel.optimizationVariableName} = ${viewModel.optimizationVariableTypeActive}(<#list viewModel.optimizationVariableDimensions as dim>${dim}<#sep>, </#list>);
          for (int i${viewModel.id} = 0; i${viewModel.id} < var${viewModel.id}.getSize(); i${viewModel.id}++)
          {
            ${viewModel.optimizationVariableName}(i${viewModel.id}) = var${viewModel.id}[i${viewModel.id}];
          }
          </#if>
          <#if viewModel.optimizationProblemType.name() == "MINIMIZATION">
          model.add(IloMinimize(env, toScalar(${viewModel.objectiveFunction})));
          <#else>
          model.add(IloMaximize(env, toScalar(${viewModel.objectiveFunction})));
          </#if>
          <#list viewModel.constraintFunctions as g>
          addConstraint(con${viewModel.id}, ${viewModel.gL[g?index]}, ${g}, ${viewModel.gU[g?index]});
          </#list>
          model.add(con${viewModel.id});

      		IloCplex cplex(model);
      		cplex.solve();

      		env.out() << "Solution status = " << cplex.getStatus() << endl;
      		env.out() << "Solution value  = " << cplex.getObjValue() << endl;

      		IloNumArray vals(env);
      		cplex.getValues(vals, var${viewModel.id});
      		env.out() << "Values        = " << vals << endl;
      		cplex.getSlacks(vals, con${viewModel.id});
      		env.out() << "Slacks        = " << vals << endl;

      		cplex.exportModel("mipex1.lp");
      	}
      	catch (IloException& e) {
      		cerr << "Concert exception caught: " << e << endl;
      	}
      	catch (...) {
      		cerr << "Unknown exception caught" << endl;
      	}

      	env.end();

      	return 0;
    }
};

#endif
