/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolic.sympy;

import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.monticar.semantics.solver.symbolic.LinearSolver;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class SympyLinearSolver implements LinearSolver {
    @Override
    public Map<String, String> solve(Set<MathAssignmentExpressionSymbol> system, Set<String> variables) {
        List<String> equationSystem = new LinkedList<>();
        for (MathAssignmentExpressionSymbol mathAssignmentExpressionSymbol : system) {

        }
        return null;
    }
}
