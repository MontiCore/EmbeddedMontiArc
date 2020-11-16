/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolic.sympy;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialGuessSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialValueSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.prettyprint.IndentPrinter;
import de.se_rwth.commons.logging.Log;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class PrintSympyFormat implements EMAMMathExpressionSymbolVisitor {

    private IndentPrinter printer = new IndentPrinter();

    private Map<String, String> renaming = new HashMap<>();
    private final String toReplace = ".";
    private final String replacment = "_";

    public String undoRenaming(String str) {
        String res = str;
        for (Map.Entry<String, String> entry : renaming.entrySet()) {
            res = res.replace(entry.getValue(), entry.getKey());
        }
        return res;
    }

    public String rename(String name) {
        if (!name.contains(toReplace)) return name;
        if (renaming.containsKey(name)) return renaming.get(name);
        String newName = name.replace(toReplace, replacment);
        renaming.put(name, newName);
        return newName;
    }

    public String print(MathExpressionSymbol node) {
        printer.clearBuffer();
        handle(node);
        return printer.getContent();
    }

    @Override
    public void traverse(MathArithmeticExpressionSymbol node) {
        handle(node.getLeftExpression());
        printer.print(node.getMathOperator());
        handle(node.getRightExpression());
    }

    @Override
    public void traverse(MathAssignmentExpressionSymbol node) {
        if (!"=".equals(node.getAssignmentOperator().getOperator()))
            notSupported(node);
        else {
            printer.print("Eq(" + rename(node.getNameOfMathValue()) + ",");
            handle(node.getExpressionSymbol());
            printer.print(")");
        }
    }

    @Override
    public void traverse(MathBooleanExpressionSymbol node) {
        notSupported(node);
//        printer.print(node.getTextualRepresentation());
    }

    @Override
    public void traverse(MathCompareExpressionSymbol node) {
//        handle(node.getLeftExpression());
//        printer.print(node.getCompareOperator());
//        handle(node.getRightExpression());
        notSupported(node);
    }

    @Override
    public void traverse(MathConditionalExpressionsSymbol node) {
//        printer.print("If:");
//        handle(node.getIfConditionalExpression());
//        if (node.getIfElseConditionalExpressions().size() > 0)
//            printer.print("Else If:");
//        for (MathConditionalExpressionSymbol symbol : node.getIfElseConditionalExpressions()) {
//            printer.print("Else If:");
//            handle(symbol);
//        }
//        if (node.getElseConditionalExpression().isPresent()) {
//            printer.print("Else:");
//            handle(node.getElseConditionalExpression().get());
//        }
        notSupported(node);
    }

    @Override
    public void traverse(MathConditionalExpressionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathForLoopHeadSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathForLoopExpressionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathNameExpressionSymbol node) {
        printer.print(rename(node.getNameToResolveValue()));
    }

    @Override
    public void traverse(MathNumberExpressionSymbol node) {
        printer.print(node.getTextualRepresentation());
    }

    @Override
    public void traverse(MathParenthesisExpressionSymbol node) {
        printer.print("(");
        handle(node.getMathExpressionSymbol());
        printer.print(")");
    }

    @Override
    public void traverse(MathPreOperatorExpressionSymbol node) {
        printer.print(node.getOperator() + "(");
        handle(node.getMathExpressionSymbol());
        printer.print(")");
    }

    @Override
    public void traverse(MathValueSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathValueType node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathMatrixAccessOperatorSymbol node) {
        printer.print(node.getAccessStartSymbol());
        int counter = 0;
        for (MathMatrixAccessSymbol accessSymbol : node.getMathMatrixAccessSymbols()) {
            handle(accessSymbol);
            ++counter;
            if (counter < node.getMathMatrixAccessSymbols().size())
                printer.print(",");
        }
        printer.print(node.getAccessEndSymbol());
    }

    @Override
    public void traverse(MathMatrixNameExpressionSymbol node) {
        printer.print(rename(node.getNameToAccess()));
        printer.print("(");
        handle(node.getMathMatrixAccessOperatorSymbol());
        printer.print(")");
    }

    @Override
    public void traverse(MathMatrixVectorExpressionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathMatrixArithmeticValueSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathMatrixAccessSymbol node) {
        if (node.getMathExpressionSymbol().isPresent())
            handle(node.getMathExpressionSymbol().get());
    }

    @Override
    public void traverse(MathMatrixArithmeticExpressionSymbol node) {
        handle(node.getLeftExpression());
        printer.print(node.getMathOperator());
        handle(node.getRightExpression());
    }

    @Override
    public void traverse(EMAMEquationSymbol node) {
        printer.print("Eq(");
        handle(node.getLeftExpression());
        printer.print(",");
        handle(node.getRightExpression());
        printer.print(")");
    }

    @Override
    public void traverse(EMAMInitialGuessSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(EMAMInitialValueSymbol node) {
        notSupported(node);
    }

    Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

    @Override
    public Set<MathExpressionSymbol> getVisitedSymbols() {
        return visitedSymbols;
    }

    private void notSupported(MathExpressionSymbol node) {
        Log.error("0xE0891239 Not supported MathExpressionSymbol: " + node.getTextualRepresentation());
    }
}
