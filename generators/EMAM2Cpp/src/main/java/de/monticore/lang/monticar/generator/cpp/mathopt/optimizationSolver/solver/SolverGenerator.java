/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.Problem;

import java.util.List;

/**
 * Abstract generator for executable code of optimization solvers.
 * Implements the bridge pattern to exchange the concrete solver generator at runtime.
 *
 */
public abstract class SolverGenerator {

    // fields
    private SolverGeneratorImplementation implementationHook;

    // getter setter

    protected SolverGeneratorImplementation getImplementationHook() {
        return implementationHook;
    }

    protected void setImplementationHook(SolverGeneratorImplementation implementationHook) {
        this.implementationHook = implementationHook;
    }

    public List<String> getNecessaryIncludes() {
        return this.implementationHook.getNecessaryIncludes();
    }

    // methods

    /**
     * Generates code from a MathOptimizationStatementSymbol to solve the described optimization problem.
     *
     * @param optimizationProblem optimization problem which should be solved
     * @param bluePrint
     * @return Executable code instruction as string
     */
    public String generateSolverInstruction(Problem optimizationProblem, List<FileContent> auxiliaryFiles, EMAMBluePrintCPP bluePrint) {
        return implementationHook.generateSolverCode(optimizationProblem, auxiliaryFiles, bluePrint);
    }
}
