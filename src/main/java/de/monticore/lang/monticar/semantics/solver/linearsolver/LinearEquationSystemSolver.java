/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.linearsolver;

import de.monticore.expressionsbasis._ast.ASTExpression;

import java.util.*;

public class LinearEquationSystemSolver {

    private final DeterminantSolver determinantSolver;
    private final Simplifier simplifier;

    public LinearEquationSystemSolver(DeterminantSolver determinantSolver, Simplifier simplifier) {
        this.determinantSolver = determinantSolver;
        this.simplifier = simplifier;
    }

    public Map<String, String> solveLinearEquationSystem(Set<ASTExpression> system, Set<String> variables) {
        Map<String, String> res = new HashMap<>();
        Map<String, Integer> mappingToIndex = new HashMap<>();

        Integer i = 0;
        for (String variable : variables) {
            mappingToIndex.put(variable, i);
            i++;
        }

        ArrayList<ArrayList<String>> A = new ArrayList<>();
        ArrayList<String> b = new ArrayList<>();

        for (ASTExpression astExpression : system) {
            RowToCoefficients converter = new RowToCoefficients(variables, mappingToIndex);
            astExpression.accept(converter);
            A.add(converter.getRow());
            b.add(converter.getSolution());
        }

        String detB = determinantSolver.det(A);

        for (String variable : variables) {
            int index = mappingToIndex.get(variable);
            String detVar = det(A, b, index);
            String eval = simplifier.simplify(detVar + "/(" + detB + ")");
            res.put(variable, eval);
        }

        return res;
    }

    private String det(ArrayList<ArrayList<String>> A, ArrayList<String> b, Integer index) {
        return determinantSolver.det(replaceColumn(A, b, index));
    }

    private ArrayList<ArrayList<String>> replaceColumn(ArrayList<ArrayList<String>> A, ArrayList<String> b, Integer index) {
        ArrayList<ArrayList<String>> res = new ArrayList<>();
        for (int i = 0; i < A.size(); i++) {
            res.add(i, new ArrayList<>());
            for (int j = 0; j < A.get(i).size(); j++) {
                if (j != index)
                    res.get(i).add(A.get(i).get(j));
                else
                    res.get(i).add(b.get(i));
            }
        }
        return res;
    }
}
