/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.analytic.z3;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.monticar.semantics.solver.analytic.AnalyticSolvers;
import de.monticore.lang.monticar.semantics.solver.analytic.UnderSpecificationSolver;
import de.monticore.lang.monticar.semantics.underspecification.SolveResult;
import de.monticore.lang.monticar.semantics.util.execution.ExecuteZ3;
import de.monticore.lang.monticar.semantics.util.parse.Z3Parser;
import de.monticore.numberunit.Rationals;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.*;

public class Z3UnderSpecificationSolver implements UnderSpecificationSolver {

    @Override
    public Map<String, String> solve(Collection<EMAMEquationSymbol> system, Collection<String> variables) {
        Map<String, String> result = new HashMap<>();

        PrintZ3Format print = new PrintZ3Format();

        Set<String> equations = new HashSet<>();
        for (EMAMEquationSymbol equationSymbol : system)
            equations.add(String.format("(assert %s)", print.print(equationSymbol)));

        Set<String> constDeclarations = new HashSet<>();
        for (String variable : print.registeredNames)
            constDeclarations.add(String.format("(declare-const %s Real)", print.rename(variable)));

        String script = String.join("\n", constDeclarations)
                + "\n" + String.join("\n", equations)
                // + add Ranges
                + "\n(check-sat)"
                + "\n(get-model)";

        File scriptFile = new File("target/temp/execute/z3/script.z3");
        try {
            scriptFile.getParentFile().mkdirs();
            scriptFile.createNewFile();
            scriptFile.setWritable(true);
            BufferedWriter writer = new BufferedWriter(new FileWriter(scriptFile));
            writer.write(script);
            writer.close();
        } catch (IOException e) {
            return result;
        }

        String output = ExecuteZ3.executeZ3(scriptFile.getPath());
        output = print.undoRenaming(output);
        SolveResult solveResult = Z3Parser.parseSolve(output);

        if (solveResult.isSatisfiable()) {
            int n = variables.size() - equations.size();
            ArrayList<String> variablesAsList = new ArrayList<>(variables);
            for (int i = 0; i < n; i++) {
                String var = variablesAsList.get(i);
                EMAMEquationSymbol emamEquationSymbol = new EMAMEquationSymbol();
                emamEquationSymbol.setLeftExpression(new MathNameExpressionSymbol(var));
                emamEquationSymbol.setRightExpression(new MathNumberExpressionSymbol(
                        Rationals.doubleToRational(solveResult.getModel().get(var))));
                system.add(emamEquationSymbol);

                return AnalyticSolvers.getNonLinearSolver().solve(system,
                        variables);
            }

        }

        return result;
    }
}
