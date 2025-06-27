/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.semantics.Constants;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.analyze.AnalyzeEquationSystemType;
import de.monticore.lang.monticar.semantics.loops.analyze.EquationSystemType;
import de.monticore.lang.monticar.semantics.loops.analyze.SpecificationConverter;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.*;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.*;

import static de.monticore.lang.monticar.semantics.loops.analyze.EquationSystemType.DAE;
import static de.monticore.lang.monticar.semantics.loops.analyze.EquationSystemType.ODE;

public class EMAEquationSystemHelper {

    public static boolean trySymbolicSolve(EMAEquationSystem system) {
        if (system.isPresentSolution()) return !system.getSolution().isEmpty();

        Optional<EMAMSpecificationSymbol> specification = SpecificationConverter.convert(system);

        if (!specification.isPresent()) return false;

        Optional<Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol>> solution =
                EquationSystemSymbolicSolver.trySymbolicSolve(
                        specification.get(), SpecificationConverter.getIncomingInformationAsVariables(system));

        if (!solution.isPresent() || solution.get().isEmpty()) return false;

        system.setSolution(convertSolutionMap(solution.get(), system.getComponentOutgoingPortInstances()));
        return true;
    }


    public static Map<EMAPortInstanceSymbol, String> convertSolutionMap(Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol> solutionForPort,
                                                                        Collection<EMAPortInstanceSymbol> ports) {
        Map<EMAPortInstanceSymbol, String> solutions = new HashMap<>();
        for (EMAMSymbolicVariableSymbol variable : solutionForPort.keySet()) {
            if (variable.isPort()) {
                Optional<EMAPortInstanceSymbol> port = ports
                        .stream()
                        .filter(p -> variable.getName().equals(NameHelper.calculateFullQualifiedNameOf(p))
                                || variable.getName().equals(p.getName()))
                        .findFirst();
                if (!port.isPresent()) Log.error("0xEMAES4016 could not find port to variable");
                else solutions.put(port.get(), solutionForPort.get(variable).getTextualRepresentation());
            }
        }
        return solutions;
    }

    public static SemiExplicitForm buildSemiExplicitForm(EMAEquationSystem system) {
        SemiExplicitFormBuilder builder = SemiExplicitFormBuilder.aSemiExplicitForm();

        Collection<EMAMSymbolicVariableSymbol> variables = SpecificationConverter.getVariables(system);
        Collection<EMAMSymbolicVariableSymbol> differentialVariables = new HashSet<>();
        for (EMAComponentInstanceSymbol component : system.getComponentInstanceSymbols()) {
            Optional<EMAMSpecificationSymbol> specification = SpecificationConverter.convert(system, component);
            if (specification.isPresent()) {
                handleSpecification(specification.get(), builder, variables, differentialVariables);
            } else {
                for (EMAPortInstanceSymbol outport : component.getOutgoingPortInstances()) {
                    builder.addG(new ComponentCall(component, outport));
                }
            }
        }

        Collection<EMAMInitialValueSymbol> initialValues = SpecificationConverter.getInitialValues(system);
        Collection<EMAMInitialGuessSymbol> initialGuesses = SpecificationConverter.getInitialGuesses(system);

        handleVariables(builder, variables, differentialVariables, initialValues, initialGuesses);

        return builder.build();
    }

    public static SemiExplicitForm buildSemiExplicitForm(EMAComponentInstanceSymbol component,
                                                         EMAMSpecificationSymbol specification) {
        SemiExplicitFormBuilder builder = SemiExplicitFormBuilder.aSemiExplicitForm();

        Collection<EMAMSymbolicVariableSymbol> variables = SpecificationConverter.getVariables(component);
        Collection<EMAMSymbolicVariableSymbol> differentialVariables = new HashSet<>();

        handleSpecification(specification, builder, variables, differentialVariables);

        Collection<EMAMInitialValueSymbol> initialValues = SpecificationConverter.getInitialValues(component);
        Collection<EMAMInitialGuessSymbol> initialGuesses = SpecificationConverter.getInitialGuesses(component);

        handleVariables(builder, variables, differentialVariables, initialValues, initialGuesses);

        return builder.build();
    }

    private static void handleSpecification(EMAMSpecificationSymbol specification, SemiExplicitFormBuilder builder, Collection<EMAMSymbolicVariableSymbol> variables, Collection<EMAMSymbolicVariableSymbol> differentialVariables) {
        for (EMAMEquationSymbol equation : specification.getEquations()) {
            EquationSystemType type = AnalyzeEquationSystemType.typeOf(equation, variables);
            if (type == ODE || type == DAE) {
                // diff(x) is alone on the left side!
                EMAMSymbolicVariableSymbol variable = getVarOfDifferentialEquation(equation, variables);
                builder.addY(variable);
                differentialVariables.add(variable);
                Optional<MathExpressionSymbol> initialValue =
                        findInitialValue(variable, specification.getInitialValues());
                if (initialValue.isPresent())
                    builder.addInitialValue(initialValue.get());
                else
                    builder.addInitialValue(new MathNumberExpressionSymbol(Rational.ZERO));
                builder.addF(equation.getRightExpression());
            } else
                builder.addG(new ExplicitFunction(equation));
        }
    }

    private static void handleVariables(SemiExplicitFormBuilder builder, Collection<EMAMSymbolicVariableSymbol> variables, Collection<EMAMSymbolicVariableSymbol> differentialVariables, Collection<EMAMInitialValueSymbol> initialValues, Collection<EMAMInitialGuessSymbol> initialGuesses) {
        variables.stream().filter(v -> !differentialVariables.contains(v))
                .forEach(v -> {
                    Optional<MathExpressionSymbol> value = findInitialValue(v, initialValues);
                    if (!value.isPresent())
                        value = findInitialGuess(v, initialGuesses);
                    if (!value.isPresent())
                        value = Optional.of(new MathNumberExpressionSymbol(Rational.ZERO));
                    builder.addZ(v);
                    builder.addInitialGuess(value.get());
                });
    }

    private static Optional<MathExpressionSymbol> findInitialValue(EMAMSymbolicVariableSymbol variable,
                                                                   Collection<EMAMInitialValueSymbol> initialValues) {
        String name = variable.getName();

        Optional<EMAMInitialValueSymbol> initialValue =
                initialValues.stream().filter(i -> i.getNameWithArray().equals(name)).findFirst();
        if (!initialValue.isPresent())
            return Optional.empty();
        else
            return Optional.ofNullable(initialValue.get().getValue());
    }

    private static Optional<MathExpressionSymbol> findInitialGuess(EMAMSymbolicVariableSymbol variable,
                                                                   Collection<EMAMInitialGuessSymbol> initialGuesses) {
        String name;
        if (variable.isPort())
            name = variable.getPort().get().getName().replace("[", "(").replace("]", ")");
        else
            name = variable.getName();

        Optional<EMAMInitialGuessSymbol> initialGuess =
                initialGuesses.stream().filter(i -> i.getNameWithArray().equals(name)).findFirst();
        if (!initialGuess.isPresent())
            return Optional.empty();
        else
            return Optional.ofNullable(initialGuess.get().getValue());
    }

    private static EMAMSymbolicVariableSymbol getVarOfDifferentialEquation(EMAMEquationSymbol equation,
                                                                           Collection<EMAMSymbolicVariableSymbol> variables) {
        MathExpressionSymbol leftExpression = equation.getLeftExpression();
        if (!(leftExpression instanceof MathMatrixNameExpressionSymbol))
            Log.error("0xEMAES4011 Differential operators are only allowed on the left side of an equation");
        String operator = ((MathMatrixNameExpressionSymbol) leftExpression).getNameToAccess();
        if (!operator.equals(Constants.derivativeOperatorName))
            Log.error("0xEMAES4012 Differential operators are only allowed on the left side of an equation");
        MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol = ((MathMatrixNameExpressionSymbol) leftExpression).getMathMatrixAccessOperatorSymbol();
        if (mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbols().size() != 1)
            Log.error("0xEMAES4013 Differential operators are only allowed on the left side of an equation");
        MathExpressionSymbol var = mathMatrixAccessOperatorSymbol.getMathMatrixAccessSymbol(0).get();
        if (!(var instanceof MathNameExpressionSymbol))
            Log.error("0xEMAES4014 Differential operators are only allowed on the left side of an equation");
        String varName = ((MathNameExpressionSymbol) var).getNameToAccess();
        Optional<EMAMSymbolicVariableSymbol> result = variables.stream().filter(v -> v.getName().equals(varName)).findFirst();
        if (!result.isPresent())
            Log.error(String.format("0xEMAES4015 Could not find variable for \"%s\" in equation %s",
                    varName, equation.getTextualRepresentation()));
        return result.get();
    }


//    public static MassMatrixRepresentation getAsMassMatrixRepresentation(EMAEquationSystem system) {
//        ArrayList<EMAPortInstanceSymbol> ports = new ArrayList<>(getPortStatements(system).keySet());
//        Set<String> variables = ports.stream().map(s -> NameHelper.calculateFullQualifiedNameOf(s)).collect(Collectors.toSet());
//
//        MathMatrixAccessOperatorSymbol x = new MathMatrixAccessOperatorSymbol();
//        MathMatrixAccessOperatorSymbol x_start = new MathMatrixAccessOperatorSymbol();
//        for (EMAPortInstanceSymbol port : ports) {
//            MathMatrixAccessSymbol var = new MathMatrixAccessSymbol(
//                    new MathNameExpressionSymbol(NameHelper.calculateFullQualifiedNameOf(port)));
//            x.addMathMatrixAccessSymbol(var);
//            if (port.isInitialGuessPresent()) {
//                MathMatrixAccessSymbol initial = new MathMatrixAccessSymbol((MathExpressionSymbol) port.getInitialGuess().getSymbol());
//                x_start.addMathMatrixAccessSymbol(initial);
//            } else {
//                MathMatrixAccessSymbol zero = new MathMatrixAccessSymbol(new MathNumberExpressionSymbol(Rational.ZERO));
//                x_start.addMathMatrixAccessSymbol(zero);
//            }
//        }
//
//        MathMatrixArithmeticValueSymbol massMatrix = new MathMatrixArithmeticValueSymbol();
//        MathMatrixAccessOperatorSymbol function = new MathMatrixAccessOperatorSymbol();
//
//        int i = 0;
//        for (EMAPortInstanceSymbol port : ports) {
//            MathMatrixAccessOperatorSymbol currentRow;
//            MathExpressionSymbol currentFunction;
//            MathAssignmentExpressionSymbol currentStatement = getPortStatements(system).get(port);
//
//            EquationSystemType loopKind = AnalyzeEquationSystemType.typeOf(portStatements.get(port), variables);
//            if (loopKind.equals(EquationSystemType.LinearDifferencial) || loopKind.equals(EquationSystemType.NonLinearDifferencial)) {
//                currentRow = buildRowFor(i, ports.size());
//                currentFunction = new MathNameExpressionSymbol(currentStatement.getNameOfMathValue());
//            } else {
//                currentRow = buildZeroRow(ports.size());
//                currentFunction = currentStatement.getExpressionSymbol();
//            }
//
//            massMatrix.addMathMatrixAccessSymbol(currentRow);
//            function.addMathMatrixAccessSymbol(new MathMatrixAccessSymbol(currentFunction));
//
//            i++;
//        }
//
//        return new MassMatrixRepresentation(massMatrix, x, x_start, function);
//
//    }
//
//    private static MathMatrixAccessOperatorSymbol buildRowFor(int index, int size) {
//        MathMatrixAccessOperatorSymbol res = new MathMatrixAccessOperatorSymbol();
//        for (int i = 0; i < size; i++) {
//            if (i == index)
//                res.addMathMatrixAccessSymbol(new MathMatrixAccessSymbol(new MathNumberExpressionSymbol(Rational.ONE)));
//            else
//                res.addMathMatrixAccessSymbol(new MathMatrixAccessSymbol(new MathNumberExpressionSymbol(Rational.ZERO)));
//        }
//        return res;
//    }
//
//    private static MathMatrixAccessOperatorSymbol buildZeroRow(int size) {
//        MathMatrixAccessOperatorSymbol res = new MathMatrixAccessOperatorSymbol();
//        for (int i = 0; i < size; i++) {
//            MathMatrixAccessSymbol zero = new MathMatrixAccessSymbol(new MathNumberExpressionSymbol(Rational.ZERO));
//            res.addMathMatrixAccessSymbol(zero);
//        }
//        return res;
//    }

}
