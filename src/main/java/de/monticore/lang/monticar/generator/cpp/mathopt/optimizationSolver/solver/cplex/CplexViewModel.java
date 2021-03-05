/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.cplex;

import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.Problem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.template.SolverViewModel;
import de.se_rwth.commons.logging.Log;

public class CplexViewModel extends SolverViewModel {

    public CplexViewModel(Problem problem) {
        super(problem);
        this.setCallSolverName("CallCplex" + problem.getId());
    }

    // methods
    //@Override
    public void setOptimizationVariableType(String optimizationVariableType) {
        //super.setOptimizationVariableType(optimizationVariableType);
        // also set active type
        if (optimizationVariableType.contentEquals("mat")) {
            this.setOptimizationVariableTypeActive("CplexMat");
        } else if (optimizationVariableType.contentEquals("colvec")) {
            this.setOptimizationVariableTypeActive("CplexMat");
        } else if (optimizationVariableType.contentEquals("double")) {
            this.setOptimizationVariableTypeActive("IloNumExprArg");
        } else {
            Log.error(String.format("Could not determine active variable type for type: \"%s\"", optimizationVariableType));
        }
    }
}
