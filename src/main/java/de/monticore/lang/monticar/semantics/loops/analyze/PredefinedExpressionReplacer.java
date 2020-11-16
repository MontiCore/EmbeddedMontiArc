/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolReplacementVisitor;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;

import java.util.Collection;
import java.util.stream.Collectors;

public class PredefinedExpressionReplacer {

    // This is something like MathFunctionFixer, should work together probably

    public static Collection<MathExpressionSymbol> replaceAll(Collection<MathExpressionSymbol> expressions) {
        return expressions.stream()
                .map(e -> replace(e))
                .collect(Collectors.toSet());
    }

    public static MathExpressionSymbol replace(MathExpressionSymbol expression) {
        MathExpressionSymbol copy = CopyEMAMMathExpressionSymbol.copy(expression);
        (new EMAMMathExpressionSymbolReplacementVisitor(PredefinedExpressionReplacer::replaceFunction)).handle(copy);
        return copy;
    }

    private static MathExpressionSymbol replaceFunction(MathExpressionSymbol expression) {
        // TODO

        if (!(expression instanceof MathMatrixNameExpressionSymbol) ||
                !((MathMatrixNameExpressionSymbol) expression).isMathMatrixAccessOperatorSymbolPresent())
            return null;

        MathMatrixNameExpressionSymbol function = (MathMatrixNameExpressionSymbol) expression;
        if (function.getNameToAccess().equals("sum") || function.getNameToAccess().equals("product")) {
            String operator = function.getNameToAccess().equals("sum") ? "+" :
                    function.getNameToAccess().equals("product") ? "*" : "";
//            ASTResolution resolution = component.getResolutionDeclarationSymbol("n").getASTResolution();
//            double n = 0;
//            if (resolution instanceof ASTUnitNumberResolution) {
//                n = ((ASTUnitNumberResolution) resolution).getNumber().get();
//            } else
//                Log.error("0x695463987 Resolution of " + component.getFullName() + " is not set");
//
//            res = calculateArithmeticOverArray((int) n, "in1", "out1", operator);
        }

        return null;
    }



}
