/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.analytic;

import de.monticore.lang.monticar.semantics.solver.analytic.sympy.SympyDAESolver;
import de.monticore.lang.monticar.semantics.solver.analytic.sympy.SympyLinearSolver;
import de.monticore.lang.monticar.semantics.solver.analytic.sympy.SympyNonLinearSolver;
import de.monticore.lang.monticar.semantics.solver.analytic.z3.Z3UnderSpecificationSolver;

public class AnalyticSolvers {

    private static LinearSolver linearSolver = new SympyLinearSolver();
    private static NonLinearSolver nonLinearSolver = new SympyNonLinearSolver();
    private static PolynomSolver polynomSolver = new SympyNonLinearSolver();
    private static ODESolver odeSolver = new SympyDAESolver();
    private static DAESolver daeSolver = new SympyDAESolver();
    private static UnderSpecificationSolver underSpecificationSolver = new Z3UnderSpecificationSolver();

    public static LinearSolver getLinearSolver() {
        return linearSolver;
    }

    public static void setLinearSolver(LinearSolver linearSolver) {
        AnalyticSolvers.linearSolver = linearSolver;
    }

    public static NonLinearSolver getNonLinearSolver() {
        return nonLinearSolver;
    }

    public static void setNonLinearSolver(NonLinearSolver nonLinearSolver) {
        AnalyticSolvers.nonLinearSolver = nonLinearSolver;
    }

    public static PolynomSolver getPolynomSolver() {
        return polynomSolver;
    }

    public static void setPolynomSolver(PolynomSolver polynomSolver) {
        AnalyticSolvers.polynomSolver = polynomSolver;
    }

    public static ODESolver getOdeSolver() {
        return odeSolver;
    }

    public static void setOdeSolver(ODESolver odeSolver) {
        AnalyticSolvers.odeSolver = odeSolver;
    }

    public static DAESolver getDaeSolver() {
        return daeSolver;
    }

    public static void setDaeSolver(DAESolver daeSolver) {
        AnalyticSolvers.daeSolver = daeSolver;
    }

    public static UnderSpecificationSolver getUnderSpecificationSolver() {
        return underSpecificationSolver;
    }

    public static void setUnderSpecificationSolver(UnderSpecificationSolver underSpecificationSolver) {
        AnalyticSolvers.underSpecificationSolver = underSpecificationSolver;
    }
}
