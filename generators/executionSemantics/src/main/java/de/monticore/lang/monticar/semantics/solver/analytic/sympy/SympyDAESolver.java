/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.analytic.sympy;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.monticar.semantics.solver.analytic.DAESolver;
import de.monticore.lang.monticar.semantics.solver.analytic.ODESolver;
import de.monticore.lang.monticar.semantics.util.execution.ExecutePython;

import java.util.*;
import java.util.stream.Collectors;

public class SympyDAESolver implements ODESolver, DAESolver {

    @Override
    public Map<String, String> solve(Collection<EMAMEquationSymbol> system, Collection<String> variables
            , Map<String, Double> startValues) {
        PrintSympyFormat print = new PrintSympyFormat();
        List<String> equationSystem = new LinkedList<>();
        for (EMAMEquationSymbol mathEquationSymbol : system)
            equationSystem.add(print.print(mathEquationSymbol));
        List<String> startValueKeys = startValues.keySet().stream().collect(Collectors.toList());
        String arguments = String.join(" ", equationSystem);
        arguments += " --functions "
                + String.join(" ", startValueKeys.stream()
                    .map(s -> print.rename(s)).collect(Collectors.toList()))
                + " " + String.join(" ", variables.stream()
                    .filter(s -> !startValueKeys.contains(s))
                    .map(s -> print.rename(s)).collect(Collectors.toList()));
        arguments += " --y0 "
                + String.join(" ", startValueKeys.stream()
                    .map(s -> startValues.get(s).toString()).collect(Collectors.toList()));
        String solutions = ExecutePython.executePython("sympy/solveDae", arguments);
        solutions = print.undoRenaming(solutions);

        Map<String, String> res = new HashMap<>();
        if (solutions.equals("-1")) return res;
        if (solutions.startsWith("Traceback")) return res;

        for (String sol : solutions.split("\r\n")) {
            String[] split = sol.split("=");
            res.put(split[0].trim(), split[1].trim());
        }
        return res;
    }

}
