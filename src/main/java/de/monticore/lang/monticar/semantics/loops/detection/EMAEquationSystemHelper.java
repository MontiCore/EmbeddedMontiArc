/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSpecificationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.analyze.SpecificationConverter;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class EMAEquationSystemHelper {

    public static boolean trySymbolicSolve(EMAEquationSystem system) {
        if (system.isPresentSolution()) return !system.getSolution().isEmpty();

        Optional<EMAMSpecificationSymbol> specification = SpecificationConverter.convert(system);

        if (!specification.isPresent()) return false;

        Optional<Map<EMAMSymbolicVariableSymbol, MathExpressionSymbol>> solution = EquationSystemSymbolicSolver.trySymbolicSolve(
                specification.get());

        if (!solution.isPresent() || solution.get().isEmpty()) return false;

        system.setSolution(convertSolutionMap(solution.get(), system.getOutgoingPortInstances()));
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
                if (!port.isPresent()) Log.error("TODO could not find port to variable");
                else solutions.put(port.get(), solutionForPort.get(variable).getTextualRepresentation());
            }
        }
        return solutions;
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
//            LoopKind loopKind = CheckKind.kindOf(portStatements.get(port), variables);
//            if (loopKind.equals(LoopKind.LinearDifferencial) || loopKind.equals(LoopKind.NonLinearDifferencial)) {
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
