/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver;

/**
 * Generates solver code which solves mixed integer quadratic optimization problems
 */
public class MIQPSolverGenerator extends SolverGenerator {

    // constructor
    public MIQPSolverGenerator(MIQPSolverGeneratorImplementation impl) {
        this.setImplementationHook(impl);
    }
}
