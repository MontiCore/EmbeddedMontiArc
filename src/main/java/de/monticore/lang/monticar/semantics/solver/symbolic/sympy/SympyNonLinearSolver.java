/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolic.sympy;

import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.monticar.semantics.solver.ExecutePython;
import de.monticore.lang.monticar.semantics.solver.symbolic.NonLinearSolver;

import java.util.*;
import java.util.stream.Collectors;

public class SympyNonLinearSolver implements NonLinearSolver {
    @Override
    public Map<String, String> solve(Set<MathAssignmentExpressionSymbol> system, Set<String> variables) {
        PrintSympyFormat print = new PrintSympyFormat();
        List<String> equationSystem = new LinkedList<>();
        for (MathAssignmentExpressionSymbol mathAssignmentExpressionSymbol : system)
            equationSystem.add(print.print(mathAssignmentExpressionSymbol));

        String arguments = String.join(" ", equationSystem)
                + " --symbols "
                + String.join(" ", variables.stream().map(s -> print.rename(s)).collect(Collectors.toList()));
        String solutions = ExecutePython.executePython("sympy/solveLinear", arguments);
        solutions = print.undoRenaming(solutions);

        Map<String, String> res = new HashMap<>();
        if (solutions.equals("-1")) return res;

        for (String sol : solutions.split("\r\n")) {
            String[] split = sol.split("=");
            res.put(split[0].trim(), split[1].trim());
        }
        return res;
    }
}
