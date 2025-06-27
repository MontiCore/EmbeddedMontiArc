/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem;

/**
 * Represents a continious non linear optimization problem of the following form:
 * min     f(x)
 * x in R^n
 * s.t.         g_L <= g(x) <= g_U
 * .            x_L <=  x   <= x_U
 * where f and g are nonlinear functions, x in R^n, f(x) in R, g(x) in R^m with
 * n dimension of x
 * m number of constraints
 * A. Wächter and L. T. Biegler, ​On the Implementation of a Primal-Dual Interior Point Filter Line Search Algorithm for Large-Scale Nonlinear Programming, Mathematical Programming 106(1), pp. 25-57, 2006
 *
 */
public class NLPProblem extends ConvexProblem {

}

