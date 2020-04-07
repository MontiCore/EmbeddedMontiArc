/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.ipopt;

import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.Problem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.template.SolverViewModel;
import de.se_rwth.commons.logging.Log;

/**
 * Contains all necessary information needed to generate a freemarker template of IPOPT C++ code.
 *
 */
public class IpoptViewModel extends SolverViewModel {

    /**
     * Reservated in IPOPT calculations as optimization variable
     */
    private static final String IPOPT_OPTIMIZATION_VAR = "x";

    /**
     * Reservated in IPOPT calculations as objective variable
     */
    private static final String IPOPT_OBJECTIVE_VAR = "fg";

    /**
     * Reservated in IPOPT calculations as constraint function pointer
     */
    private static final String IPOPT_CONSTRAINT_FUNCTION_VAR = "fg";

    /**
     * array of all reservated variables used by IPOPT
     */
    private static final String[] IPOPT_RESERVATED_VARS = {IPOPT_OPTIMIZATION_VAR, IPOPT_OBJECTIVE_VAR, IPOPT_CONSTRAINT_FUNCTION_VAR};

    // constructor

    /**
     * Generated a IPOPT view model from a optimization problem
     *
     * @param problem non linear optimization problem
     */
    public IpoptViewModel(Problem problem) {
        super(problem);
        this.setCallSolverName("CallIpopt" + problem.getId());
    }

    // methods
    @Override
    public void setOptimizationVariableType(String optimizationVariableType) {
        super.setOptimizationVariableType(optimizationVariableType);
        // also set active type
        if (optimizationVariableType.contentEquals("mat")) {
            this.setOptimizationVariableTypeActive("ADMat");
        } else if (optimizationVariableType.contentEquals("colvec")) {
            this.setOptimizationVariableTypeActive("ADMat");
        } else if (optimizationVariableType.contentEquals("double")) {
            this.setOptimizationVariableTypeActive("AD<double>");
        } else {
            Log.error(String.format("Could not determine active variable type for type: \"%s\"", optimizationVariableType));
        }
    }

    public void resolveIpoptNameConflicts() {
        for (String reservated : IPOPT_RESERVATED_VARS) {
            if (containsVariable(reservated)) {
                String replacementVar = findRelpacementVariable(reservated);
                replaceVariable(reservated, replacementVar);
                if (getOptimizationVariableName().contentEquals(reservated))
                    setOptimizationVariableName(replacementVar);
                if (getObjectiveVariableName().contentEquals(reservated))
                    setObjectiveVariableName(replacementVar);
            }
        }
    }

}
