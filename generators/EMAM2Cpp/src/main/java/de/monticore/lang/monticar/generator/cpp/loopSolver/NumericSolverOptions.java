/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

public class NumericSolverOptions {
    // Absolute tolerance for the solver
    public static double ATOL = 1.0E-15;

    // Relative tolerance for the solver
    public static double RTOL = 1.0E-15;

    // Tolerance for the numerical Jacobian
    public static double JTOL = 1.0E-6;

    // Level for logging, if it is supported
    public static int LEVEL_LOGGING = 0;

    // Delta t used in the solver, lower delta t leads to longer execution times
    //  but better accuracy
    public static double DT_SOLVER = 1.0E-2;
}
