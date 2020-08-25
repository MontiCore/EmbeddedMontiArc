/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver;

/**
 * Generates solver code which solves quadratic optimization problems
 */
public class QPSolverGenerator extends SolverGenerator {

    // constructor
    public QPSolverGenerator(QPSolverGeneratorImplementation impl) {
        this.setImplementationHook(impl);
    }
}
