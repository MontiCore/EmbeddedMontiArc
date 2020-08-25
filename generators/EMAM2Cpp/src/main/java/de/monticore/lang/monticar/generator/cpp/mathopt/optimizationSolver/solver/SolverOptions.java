/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver;

import java.util.HashMap;

public class SolverOptions extends HashMap<String, String> {

    private Solver solver;

    public static SolverOptions getIpoptDefaultOptions() {
        SolverOptions result = new SolverOptions();
        result.solver = Solver.Ipopt;
        result.put("Integer print_level", "1");
        result.put("String  sb", "yes");
        result.put("Integer max_iter", "500");
        result.put("Numeric tol", "1e-6");
        result.put("String  derivative_test", "second-order");
        result.put("Numeric point_perturbation_radius", "0.");
        result.put("Retape", "false");
        return result;
    }
}
