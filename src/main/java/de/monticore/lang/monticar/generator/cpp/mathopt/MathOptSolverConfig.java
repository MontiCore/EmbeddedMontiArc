/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt;

import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.Solver;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.SolverOptions;

public class MathOptSolverConfig {

    private Solver preferedSolver = Solver.none;
    private SolverOptions solverOptions = new SolverOptions();
    private boolean forceUsePreferredSolver = false;

    public Solver getPreferedSolver() {
        return preferedSolver;
    }

    public void setPreferedSolver(Solver preferedSolver) {
        this.preferedSolver = preferedSolver;
    }

    public SolverOptions getSolverOptions() {
        return solverOptions;
    }

    public void setSolverOptions(SolverOptions solverOptions) {
        this.solverOptions = solverOptions;
    }

    public boolean isForceUsePreferredSolver() {
        return forceUsePreferredSolver;
    }

    public void setForceUsePreferredSolver(boolean forceUsePreferredSolver) {
        this.forceUsePreferredSolver = forceUsePreferredSolver;
    }
}
