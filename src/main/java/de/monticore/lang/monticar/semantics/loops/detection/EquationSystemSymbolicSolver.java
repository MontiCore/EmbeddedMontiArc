/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.analyze.CheckKind;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopKind;
import de.monticore.lang.monticar.semantics.solver.symbolic.SymbolicSolvers;
import de.monticore.lang.monticar.semantics.util.math.MathHelper;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class EquationSystemSymbolicSolver {

    public static Optional<Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol>> trySymbolicSolve(EMAMSpecificationSymbol specification) {

        LoopKind kind = CheckKind.kindOf(specification.getEquations(), specification.getVariables());

        EquationSystemSymbolicSolver solver = new EquationSystemSymbolicSolver(specification);

        Map<String, String> solution = new HashMap<>();
        switch (kind) {
            case Linear:
                solution = solver.handleLinearSymbolic();
                break;
            case LinearDifferencial:
                solution = solver.handleDAESymbolic();
                break;
            case NonLinearDifferencial:
                solution = solver.handleDAESymbolic();
                break;
            case NonLinear:
                solution = solver.handleNonLinearSymbolic();
                break;
            case Polynom:
                solution = solver.handlePolynomSymbolic();
                break;
            default:
                Log.warn("0x907651 not yet supported");
        }

        if (solution.isEmpty())
            return Optional.empty();

        return convertSolutionToSymbols(solution, specification);
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
        return SymbolicSolvers.getDaeSolver().solve(equations,
                variables.stream().map(v -> v.getName()).collect(Collectors.toSet()),
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
        return SymbolicSolvers.getOdeSolver().solve(equations,
                variables.stream().map(v -> v.getName()).collect(Collectors.toSet()),
                startValues);
    }

    private Map<String, String> handleLinearSymbolic() {
        // Symbolic Solve
        return SymbolicSolvers.getLinearSolver().solve(equations,
                variables.stream().map(v -> v.getName()).collect(Collectors.toSet()));
    }

    private Map<String, String> handleNonLinearSymbolic() {
        // Symbolic Solve
        return SymbolicSolvers.getNonLinearSolver().solve(equations,
                variables.stream().map(v -> v.getName()).collect(Collectors.toSet()));
    }

    private Map<String, String> handlePolynomSymbolic() {
        // Symbolic Solve
        return SymbolicSolvers.getPolynomSolver().solve(equations,
                variables.stream().map(v -> v.getName()).collect(Collectors.toSet()));
    }


    private static Set<String> getVariablesAsStringSet(Collection<EMAPortInstanceSymbol> variables) {
        return variables
                .stream()
                .map(NameHelper::calculateFullQualifiedNameOf)
                .collect(Collectors.toSet());
    }

}
