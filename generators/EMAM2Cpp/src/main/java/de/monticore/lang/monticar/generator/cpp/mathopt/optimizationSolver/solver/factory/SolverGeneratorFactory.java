/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.factory;

import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.*;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.*;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.cplex.CplexSolverGeneratorImplementation;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.ipopt.IpoptSolverGeneratorImplementation;
import de.se_rwth.commons.logging.Log;

/**
 * Factory to produce solver generators together with their implementation
 *
 */
public class SolverGeneratorFactory {
    private static SolverGeneratorFactory ourInstance = new SolverGeneratorFactory();

    private SolverGeneratorFactory() {
    }

    public static SolverGeneratorFactory getInstance() {
        return ourInstance;
    }

    /**
     * Creates the prefered solver for the optimization problem. Returns the default solver if problem can not be handled by prefered solver
     *
     * @param problem         any problem
     * @param preferredSolver prefered solver
     * @return the prefered solver generator
     */
    public SolverGenerator createPreferredSolverForProblem(Problem problem, Solver preferredSolver, boolean forceUsePreferredSolver) {
        SolverGenerator result = createDefaultSolverForProblem(problem);
        switch (preferredSolver) {
            case Ipopt:
                if (problem instanceof DNLPProblem || problem instanceof NLPProblem || forceUsePreferredSolver) {
                    NLPSolverGeneratorImplementation impl = new IpoptSolverGeneratorImplementation();
                    result = new NLPSolverGenerator(impl);
                    if (!(problem instanceof DNLPProblem || problem instanceof NLPProblem))
                        Log.warn(String.format("Prefered solver Ipopt can not handle problem of type %s.", problem.getClass().toString()));
                } else {
                    Log.warn(String.format("Prefered solver Ipopt can not handle problem of type %s. Default solver will be returned instead.", problem.getClass().toString()));
                }
                break;
            case Cplex:
                if (problem instanceof MIQPProblem) {
                    MIQPSolverGeneratorImplementation impl = new CplexSolverGeneratorImplementation();
                    result = new MIQPSolverGenerator(impl);
                } else if (problem instanceof QPProblem || forceUsePreferredSolver) {
                    QPSolverGeneratorImplementation impl = new CplexSolverGeneratorImplementation();
                    result = new QPSolverGenerator(impl);
                    if (!(problem instanceof DNLPProblem))
                        Log.warn(String.format("Prefered solver CPLEX can not handle problem of type %s.", problem.getClass().toString()));
                } else {
                    Log.warn(String.format("Prefered solver CPLEX can not handle problem of type %s. Default solver will be returned instead.", problem.getClass().toString()));
                }
                break;
            case none:
                break;
            default:
                Log.warn(String.format("Prefered solver %s not found. Default solver will be returned instead.", preferredSolver.toString()));
                break;
        }
        return result;
    }

    /**
     * Default solver for all problems is Ipopt
     *
     * @param problem any optimiaztion problem
     * @return default solver generator
     */
    public SolverGenerator createDefaultSolverForProblem(Problem problem) {
        SolverGenerator result = null;
        if (problem instanceof NLPProblem) {
            NLPSolverGeneratorImplementation impl = new IpoptSolverGeneratorImplementation();
            result = new NLPSolverGenerator(impl);
        } else {
            Log.error(String.format("No solver found for problem class %s", problem.getClass().toString()));
        }
        return result;
    }
}
