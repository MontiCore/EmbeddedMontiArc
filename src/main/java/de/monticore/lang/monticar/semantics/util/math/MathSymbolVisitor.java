/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.math;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;

public interface MathSymbolVisitor {
    public default void handle(MathExpressionSymbol node) {
        if (node instanceof MathArithmeticExpressionSymbol)
            handle((MathArithmeticExpressionSymbol) node);
        else if (node instanceof MathAssignmentExpressionSymbol)
            handle((MathAssignmentExpressionSymbol) node);
        else if (node instanceof MathBooleanExpressionSymbol)
            handle((MathBooleanExpressionSymbol) node);
        else if (node instanceof MathCompareExpressionSymbol)
            handle((MathCompareExpressionSymbol) node);
        else if (node instanceof MathConditionalExpressionsSymbol)
            handle((MathConditionalExpressionsSymbol) node);
        else if (node instanceof MathConditionalExpressionSymbol)
            handle((MathConditionalExpressionSymbol) node);
        else if (node instanceof MathForLoopHeadSymbol)
            handle((MathForLoopHeadSymbol) node);
        else if (node instanceof MathForLoopExpressionSymbol)
            handle((MathForLoopExpressionSymbol) node);
        else if (node instanceof MathNameExpressionSymbol)
            handle((MathNameExpressionSymbol) node);
        else if (node instanceof MathNumberExpressionSymbol)
            handle((MathNumberExpressionSymbol) node);
        else if (node instanceof MathParenthesisExpressionSymbol)
            handle((MathParenthesisExpressionSymbol) node);
        else if (node instanceof MathPreOperatorExpressionSymbol)
            handle((MathPreOperatorExpressionSymbol) node);
        else if (node instanceof MathValueSymbol)
            handle((MathValueSymbol) node);
        else {

        }
    }

    public default void handle(MathArithmeticExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathArithmeticExpressionSymbol node) {

    }

    public default void traverse(MathArithmeticExpressionSymbol node) {
        handle(node.getLeftExpression());
        handle(node.getRightExpression());
    }

    public default void endVisit(MathArithmeticExpressionSymbol node) {

    }

    public default void handle(MathAssignmentExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathAssignmentExpressionSymbol node) {

    }

    public default void traverse(MathAssignmentExpressionSymbol node) {
        handle(node.getExpressionSymbol());
    }

    public default void endVisit(MathAssignmentExpressionSymbol node) {

    }

    public default void handle(MathBooleanExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathBooleanExpressionSymbol node) {

    }

    public default void traverse(MathBooleanExpressionSymbol node) {

    }

    public default void endVisit(MathBooleanExpressionSymbol node) {

    }

    public default void handle(MathCompareExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathCompareExpressionSymbol node) {

    }

    public default void traverse(MathCompareExpressionSymbol node) {
        handle(node.getLeftExpression());
        handle(node.getRightExpression());
    }

    public default void endVisit(MathCompareExpressionSymbol node) {

    }

    public default void handle(MathConditionalExpressionsSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathConditionalExpressionsSymbol node) {

    }

    public default void traverse(MathConditionalExpressionsSymbol node) {
        handle(node.getIfConditionalExpression());
        for (MathConditionalExpressionSymbol ifElseConditionalExpression : node.getIfElseConditionalExpressions()) {
            handle(ifElseConditionalExpression);
        }
        if (node.getElseConditionalExpression().isPresent())
            handle(node.getElseConditionalExpression().get());
    }

    public default void endVisit(MathConditionalExpressionsSymbol node) {

    }

    public default void handle(MathConditionalExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathConditionalExpressionSymbol node) {

    }

    public default void traverse(MathConditionalExpressionSymbol node) {
        if (node.getCondition().isPresent())
            handle(node.getCondition().get());
        for (MathExpressionSymbol bodyExpression : node.getBodyExpressions()) {
            handle(bodyExpression);
        }
    }

    public default void endVisit(MathConditionalExpressionSymbol node) {

    }

    public default void handle(MathForLoopHeadSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathForLoopHeadSymbol node) {

    }

    public default void traverse(MathForLoopHeadSymbol node) {
        handle(node.getMathExpression());
    }

    public default void endVisit(MathForLoopHeadSymbol node) {

    }

    public default void handle(MathForLoopExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathForLoopExpressionSymbol node) {

    }

    public default void traverse(MathForLoopExpressionSymbol node) {
        handle(node.getForLoopHead());
        for (MathExpressionSymbol mathExpressionSymbol : node.getForLoopBody()) {
            handle(mathExpressionSymbol);
        }
    }

    public default void endVisit(MathForLoopExpressionSymbol node) {

    }

    public default void handle(MathNameExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathNameExpressionSymbol node) {

    }

    public default void traverse(MathNameExpressionSymbol node) {

    }

    public default void endVisit(MathNameExpressionSymbol node) {

    }

    public default void handle(MathNumberExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathNumberExpressionSymbol node) {

    }

    public default void traverse(MathNumberExpressionSymbol node) {

    }

    public default void endVisit(MathNumberExpressionSymbol node) {

    }

    public default void handle(MathParenthesisExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathParenthesisExpressionSymbol node) {

    }

    public default void traverse(MathParenthesisExpressionSymbol node) {
        handle(node.getMathExpressionSymbol());
    }

    public default void endVisit(MathParenthesisExpressionSymbol node) {

    }

    public default void handle(MathPreOperatorExpressionSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathPreOperatorExpressionSymbol node) {

    }

    public default void traverse(MathPreOperatorExpressionSymbol node) {
        handle(node.getMathExpressionSymbol());
    }

    public default void endVisit(MathPreOperatorExpressionSymbol node) {

    }

    public default void handle(MathValueSymbol node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathValueSymbol node) {

    }

    public default void traverse(MathValueSymbol node) {
        handle(node.getType());
        handle(node.getValue());
    }

    public default void endVisit(MathValueSymbol node) {

    }

    public default void handle(MathValueType node) {
        visit(node);
        traverse(node);
        endVisit(node);
    }

    public default void visit(MathValueType node) {

    }

    public default void traverse(MathValueType node) {

    }

    public default void endVisit(MathValueType node) {

    }
}
