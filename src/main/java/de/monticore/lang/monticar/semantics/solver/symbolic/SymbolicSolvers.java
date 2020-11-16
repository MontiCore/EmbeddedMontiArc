/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolic;

import de.monticore.lang.monticar.semantics.solver.symbolic.sympy.SympyDAESolver;
import de.monticore.lang.monticar.semantics.solver.symbolic.sympy.SympyLinearSolver;
import de.monticore.lang.monticar.semantics.solver.symbolic.sympy.SympyNonLinearSolver;

public class SymbolicSolvers {

    private static LinearSolver linearSolver = new SympyLinearSolver();
    private static NonLinearSolver nonLinearSolver = new SympyNonLinearSolver();
    private static PolynomSolver polynomSolver = new SympyNonLinearSolver();
    private static ODESolver odeSolver = new SympyDAESolver();
    private static DAESolver daeSolver = new SympyDAESolver();

    public static LinearSolver getLinearSolver() {
        return linearSolver;
    }

    public static void setLinearSolver(LinearSolver linearSolver) {
        SymbolicSolvers.linearSolver = linearSolver;
    }

    public static NonLinearSolver getNonLinearSolver() {
        return nonLinearSolver;
    }

    public static void setNonLinearSolver(NonLinearSolver nonLinearSolver) {
        SymbolicSolvers.nonLinearSolver = nonLinearSolver;
    }

    public static PolynomSolver getPolynomSolver() {
        return polynomSolver;
    }

    public static void setPolynomSolver(PolynomSolver polynomSolver) {
        SymbolicSolvers.polynomSolver = polynomSolver;
    }

    public static ODESolver getOdeSolver() {
        return odeSolver;
    }

    public static void setOdeSolver(ODESolver odeSolver) {
        SymbolicSolvers.odeSolver = odeSolver;
    }

    public static DAESolver getDaeSolver() {
        return daeSolver;
    }

    public static void setDaeSolver(DAESolver daeSolver) {
        SymbolicSolvers.daeSolver = daeSolver;
    }


}
