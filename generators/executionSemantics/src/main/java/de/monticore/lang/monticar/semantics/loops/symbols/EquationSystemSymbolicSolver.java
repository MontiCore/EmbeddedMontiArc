/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.semantics.loops.analyze.AnalyzeEquationSystemType;
import de.monticore.lang.monticar.semantics.loops.analyze.EquationSystemType;
import de.monticore.lang.monticar.semantics.solver.analytic.AnalyticSolvers;
import de.monticore.lang.monticar.semantics.solver.solutionValidation.SolutionValidation;
import de.monticore.lang.monticar.semantics.util.math.MathHelper;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class EquationSystemSymbolicSolver {

    public static Optional<Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol>>
    trySymbolicSolve(EMAMSpecificationSymbol specification, Collection<EMAMSymbolicVariableSymbol> inports) {

        EquationSystemType type = AnalyzeEquationSystemType.typeOf(specification.getEquations(), specification.getVariables());

        EquationSystemSymbolicSolver solver = new EquationSystemSymbolicSolver(specification);

        Map<String, String> solution = new HashMap<>();
        switch (type) {
            case Linear:
                solution = solver.handleLinearSymbolic();
                break;
            case ODE:
                solution = solver.handleODESymbolic();
                break;
            case DAE:
                solution = solver.handleDAESymbolic();
                break;
            case NonLinear:
                solution = solver.handleNonLinearSymbolic();
                break;
            case Polynom:
                solution = solver.handlePolynomSymbolic();
                break;
            case Underspecified:
                solution = solver.handleUnderSpecification();
                break;
            default:
                Log.warn("0x907651 not yet supported");
        }

        if (solution.isEmpty())
            return Optional.empty();

        Optional<Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol>> solutionAsSymbols =
                convertSolutionToSymbols(solution, specification);

        if (SolutionValidation.isValid(specification.getVariables(), inports, solutionAsSymbols.get()))
            return solutionAsSymbols;

        return Optional.empty();
    }

    private static Optional<Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol>> convertSolutionToSymbols(Map<String, String> solution, EMAMSpecificationSymbol specification) {
        Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol> result = new HashMap<>();
        for (EMAMSymbolicVariableSymbol variable : specification.getVariables()) {
            if (!solution.containsKey(variable.getName())) return Optional.empty();
            MathExpressionSymbol symbol = MathHelper.getSymbolFromString(solution.get(variable.getName()));
            result.put(variable, symbol);
        }

        return Optional.of(result);
    }

    private final Collection<EMAMEquationSymbol> equations;
    private final Collection<EMAMSymbolicVariableSymbol> variables;
    private final Collection<EMAMInitialGuessSymbol> initialGuesses;
    private final Collection<EMAMInitialValueSymbol> initialValues;

    public EquationSystemSymbolicSolver(EMAMSpecificationSymbol specification) {
        this.equations = specification.getEquations();
        this.variables = specification.getVariables();
        this.initialGuesses = specification.getInitialGuesses();
        this.initialValues = specification.getInitialValues();
    }

    public EquationSystemSymbolicSolver(Collection<EMAMEquationSymbol> equations,
                                        Collection<EMAMSymbolicVariableSymbol> variables,
                                        Collection<EMAMInitialGuessSymbol> initialGuesses,
                                        Collection<EMAMInitialValueSymbol> initialValues) {

        this.equations = new HashSet<>(equations);
        this.variables = new HashSet<>(variables);
        this.initialGuesses = new HashSet<>(initialGuesses);
        this.initialValues = new HashSet<>(initialValues);
    }

    private Map<String, String> handleDAESymbolic() {
        Map<String, Double> startValues = new HashMap<>();

        initialGuesses.stream().forEach(i ->
                startValues.put(i.getNameWithArray(),
                        Double.parseDouble(i.getValue().getTextualRepresentation())));

        initialValues.stream().forEach(i ->
                startValues.put(i.getNameWithArray(),
                        Double.parseDouble(i.getValue().getTextualRepresentation())));

        for (EMAMSymbolicVariableSymbol variable : variables)
            if (!startValues.containsKey(variable.getName()))
                startValues.put(variable.getName(), 0.0);

        // Symbolic Solve
        return AnalyticSolvers.getDaeSolver().solve(equations,
                getVariablesAsStringSet(),
                startValues);
    }


    private Map<String, String> handleODESymbolic() {
        Map<String, Double> startValues = new HashMap<>();

        initialGuesses.stream().forEach(i ->
                startValues.put(i.getNameWithArray(),
                        Double.parseDouble(i.getValue().getTextualRepresentation())));

        initialValues.stream().forEach(i ->
                startValues.put(i.getNameWithArray(),
                        Double.parseDouble(i.getValue().getTextualRepresentation())));

        for (EMAMSymbolicVariableSymbol variable : variables)
            if (!startValues.containsKey(variable.getName()))
                startValues.put(variable.getName(), 0.0);

        // Symbolic Solve
        return AnalyticSolvers.getOdeSolver().solve(equations,
                getVariablesAsStringSet(),
                startValues);
    }

    private Map<String, String> handleLinearSymbolic() {
        // Symbolic Solve
        return AnalyticSolvers.getLinearSolver().solve(equations,
                getVariablesAsStringSet());
    }

    private Map<String, String> handleNonLinearSymbolic() {
        // Symbolic Solve
        return AnalyticSolvers.getNonLinearSolver().solve(equations,
                getVariablesAsStringSet());
    }

    private Map<String, String> handlePolynomSymbolic() {
        // Symbolic Solve
        return AnalyticSolvers.getPolynomSolver().solve(equations,
                getVariablesAsStringSet());
    }

    private Map<String, String> handleUnderSpecification() {
        return AnalyticSolvers.getUnderSpecificationSolver().solve(equations,
                getVariablesAsStringSet());
    }


    private Set<String> getVariablesAsStringSet() {
        return variables.stream().map(v -> v.getName()).collect(Collectors.toSet());
    }
}
