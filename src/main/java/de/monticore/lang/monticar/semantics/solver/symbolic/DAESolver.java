/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolic;

import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;

import java.util.Map;
import java.util.Set;

public interface DAESolver {
    public Map<String, String> solve(Set<MathAssignmentExpressionSymbol> system, Set<String> variables
            , Map<String, Double> startValues);
}
