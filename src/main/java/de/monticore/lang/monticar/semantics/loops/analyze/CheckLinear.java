/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;

import java.util.Set;

public class CheckLinear extends MathExpressionSymbolVisitor {
    private boolean result = true;

    private final Set<String> variables;

    public CheckLinear(Set<String> variables) {
        this.variables = variables;
    }

    public boolean getResult() {
        return result;
    }

    @Override
    public void visit(MathArithmeticExpressionSymbol node) {
        if (!node.getOperator().equals("+") && !node.getOperator().equals("-")) {
            if (ContainsName.containsName(node.getLeftExpression(), variables)
                    && ContainsName.containsName(node.getRightExpression(), variables)) {
                result = false;
            }
        }
    }

    @Override
    public void visit(MathAssignmentExpressionSymbol node) {
        if (variables.contains(node.getNameOfMathValue())
                && !node.getAssignmentOperator().getOperator().equals("+=")
                && !node.getAssignmentOperator().getOperator().equals("-=")
                && !node.getAssignmentOperator().getOperator().equals("=")) {
            if (ContainsName.containsName(node.getExpressionSymbol(), variables)) {
                result = false;
            }
        }
    }

    @Override
    public void visit(MathBooleanExpressionSymbol node) {
        // TODO
        result = false;
    }

    @Override
    public void visit(MathCompareExpressionSymbol node) {
        // TODO
        result = false;
    }

    @Override
    public void visit(MathConditionalExpressionsSymbol node) {
        // TODO
        result = false;
    }

    @Override
    public void visit(MathConditionalExpressionSymbol node) {
        // TODO
        result = false;
    }

    @Override
    public void visit(MathForLoopHeadSymbol node) {
        // TODO
        result = false;
    }

    @Override
    public void visit(MathForLoopExpressionSymbol node) {
        // TODO
        result = false;
    }

    @Override
    public void visit(MathNameExpressionSymbol node) {

    }

    @Override
    public void visit(MathNumberExpressionSymbol node) {

    }

    @Override
    public void visit(MathParenthesisExpressionSymbol node) {

    }

    @Override
    public void visit(MathPreOperatorExpressionSymbol node) {
        // TODO
        if (!node.getOperator().equals("+") && !node.getOperator().equals("-"))
            result = false;
    }

    @Override
    public void visit(MathValueSymbol node) {

    }

    @Override
    public void visit(MathValueType node) {

    }
}
