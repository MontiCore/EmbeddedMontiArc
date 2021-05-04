/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.analytic.z3;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.prettyprint.IndentPrinter;
import de.se_rwth.commons.logging.Log;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class PrintZ3Format implements EMAMMathExpressionSymbolVisitor {

    public Set<String> registeredNames = new HashSet<>();

    private IndentPrinter printer = new IndentPrinter();

    private Map<String, String> renaming = new HashMap<>();
    private final String toReplace = ".";
    private final String replacment = "_";

    boolean hasErrors = false;

    public String undoRenaming(String str) {
        String res = str;
        for (Map.Entry<String, String> entry : renaming.entrySet()) {
            res = res.replace(entry.getValue(), entry.getKey());
        }
        return res;
    }

    public String rename(String name) {
        if (!name.contains(toReplace)) {
            registeredNames.add(name);
            return name;
        }
        if (renaming.containsKey(name)) return renaming.get(name);
        String newName = name.replace(toReplace, replacment);
        renaming.put(name, newName);
        registeredNames.add(newName);
        return newName;
    }

    public String print(MathExpressionSymbol node) {
        printer.clearBuffer();
        hasErrors = false;
        handle(node);
        if (hasErrors) return "";
        return printer.getContent();
    }

    @Override
    public void traverse(EMAMSpecificationSymbol node) {
    }

    @Override
    public void traverse(EMAMSymbolicVariableSymbol node) {
    }

    @Override
    public void traverse(EMAMEquationSymbol node) {
        printer.print("(= ");
        handle(node.getLeftExpression());
        printer.print(" ");
        handle(node.getRightExpression());
        printer.print(")");
    }

    @Override
    public void traverse(EMAMInitialGuessSymbol node) {
    }

    @Override
    public void traverse(EMAMInitialValueSymbol node) {
    }

    @Override
    public void traverse(MathStringExpression node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathChainedExpression node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathOptimizationStatementSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathOptimizationConditionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathArithmeticExpressionSymbol node) {
        printer.print(String.format("(%s ", node.getMathOperator()));
        handle(node.getLeftExpression());
        printer.print(" ");
        handle(node.getRightExpression());
        printer.print(")");
    }

    @Override
    public void traverse(MathAssignmentExpressionSymbol node) {
        printer.print(String.format("(%s %s ", node.getAssignmentOperator().getOperator(),
                rename(node.getNameOfMathValue())));
        handle(node.getExpressionSymbol());
        printer.print(")");
    }

    @Override
    public void traverse(MathBooleanExpressionSymbol node) {
        printer.print(node.getTextualRepresentation());
    }

    @Override
    public void traverse(MathCompareExpressionSymbol node) {
        printer.print(String.format("(%s ", node.getOperator()));
        handle(node.getLeftExpression());
        printer.print(" ");
        handle(node.getRightExpression());
        printer.print(")");
    }

    @Override
    public void traverse(MathConditionalExpressionsSymbol node) {
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
        printer.print(rename(node.getNameToAccess()));
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
        traverse(node, null);
    }

    public void traverse(MathMatrixAccessOperatorSymbol node, String operator) {
        if (operator == null)
            printer.print(node.getAccessStartSymbol());
        int counter = 0;
        for (MathMatrixAccessSymbol accessSymbol : node.getMathMatrixAccessSymbols()) {
            handle(accessSymbol);
            ++counter;
            if (counter < node.getMathMatrixAccessSymbols().size())
                printer.print(operator == null ? "," : operator);
        }
        if (operator == null)
            printer.print(node.getAccessEndSymbol());
    }


    @Override
    public void traverse(MathMatrixNameExpressionSymbol node) {
        if (node.getNameToAccess().equals("sum")) {
            traverse(node.getMathMatrixAccessOperatorSymbol(), "+");
        } else if (node.getNameToAccess().equals("product")) {
            traverse(node.getMathMatrixAccessOperatorSymbol(), "*");
        } else {
            printer.print(rename(node.getNameToAccess()));
            handle(node.getMathMatrixAccessOperatorSymbol());
        }
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
        notSupported(node);
    }

    private Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

    @Override
    public Set<MathExpressionSymbol> getVisitedSymbols() {
        return this.visitedSymbols;
    }

    private void notSupported(MathExpressionSymbol node) {
        hasErrors = true;
        Log.warn("0xE0891240 Not supported MathExpressionSymbol: " + node.getTextualRepresentation());
    }
}
