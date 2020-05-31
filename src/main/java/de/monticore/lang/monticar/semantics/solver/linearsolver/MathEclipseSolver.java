/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.linearsolver;

import org.matheclipse.core.eval.ExprEvaluator;
import org.matheclipse.core.interfaces.IExpr;

import java.util.List;
import java.util.stream.Collectors;

public class MathEclipseSolver implements DeterminantSolver, Simplifier{

    private ExprEvaluator util = new ExprEvaluator();

    @Override
    public String det(List<? extends List<String>> A) {
        String mat = printMatrix(A);

        IExpr eval = util.eval("Det(" + mat + ")");
        return eval.toString();
    }

    private String printMatrix(List<? extends List<String>> A) {
        String result = A.stream()
                .map(n -> n.stream()
                        .collect(Collectors.joining(",", "{", "}")))
                .collect(Collectors.joining(",", "{", "}"));

        return result;
    }

    @Override
    public String simplify(String expression) {
        IExpr eval = util.eval("Simplify(" + expression + ")");
        return eval.toString();
    }
}
