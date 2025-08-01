/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.visitor;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;

import java.util.Set;

public interface MathExpressionSymbolVisitor {
    
    public default void handle(MathExpressionSymbol node) {
        if (node == null) return;
        else if (node instanceof MathArithmeticExpressionSymbol)
            handle((MathArithmeticExpressionSymbol) node);
        else if (node instanceof MathAssignmentExpressionSymbol)
            handle((MathAssignmentExpressionSymbol) node);
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
        else if (node instanceof MathParenthesisExpressionSymbol)
            handle((MathParenthesisExpressionSymbol) node);
        else if (node instanceof MathPreOperatorExpressionSymbol)
            handle((MathPreOperatorExpressionSymbol) node);
        else if (node instanceof MathValueType)
            handle((MathValueType) node);
        else if (node instanceof MathNameExpressionSymbol)
            handle((MathNameExpressionSymbol) node);
        else if (node instanceof MathValueSymbol)
            handle((MathValueSymbol) node);
        else if (node instanceof MathNumberExpressionSymbol)
            handle((MathNumberExpressionSymbol) node);
        else if (node instanceof MathBooleanExpressionSymbol)
            handle((MathBooleanExpressionSymbol) node);
        else if (node instanceof MathMatrixAccessOperatorSymbol)
            handle((MathMatrixAccessOperatorSymbol) node);
        else if (node instanceof MathMatrixNameExpressionSymbol)
            handle((MathMatrixNameExpressionSymbol) node);
        else if (node instanceof MathMatrixVectorExpressionSymbol)
            handle((MathMatrixVectorExpressionSymbol) node);
        else if (node instanceof MathMatrixArithmeticValueSymbol)
            handle((MathMatrixArithmeticValueSymbol) node);
        else if (node instanceof MathMatrixAccessSymbol)
            handle((MathMatrixAccessSymbol) node);
        else if (node instanceof MathMatrixArithmeticExpressionSymbol)
            handle((MathMatrixArithmeticExpressionSymbol) node);
        else
            handleUnkown(node);
    }

    public default void handleUnkown(MathExpressionSymbol node) {

    }

    public default void handle(MathArithmeticExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathArithmeticExpressionSymbol node) {

    }

    public default void traverse(MathArithmeticExpressionSymbol node) {
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        if (node.getRightExpression() != null) handle(node.getRightExpression());
    }

    public default void endVisit(MathArithmeticExpressionSymbol node) {

    }

    public default void handle(MathAssignmentExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathAssignmentExpressionSymbol node) {

    }

    public default void traverse(MathAssignmentExpressionSymbol node) {
        if (node.getMathMatrixAccessOperatorSymbol() != null) handle(node.getMathMatrixAccessOperatorSymbol());
        if (node.getExpressionSymbol() != null) handle(node.getExpressionSymbol());
    }

    public default void endVisit(MathAssignmentExpressionSymbol node) {

    }

    public default void handle(MathBooleanExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathBooleanExpressionSymbol node) {

    }

    public default void traverse(MathBooleanExpressionSymbol node) {

    }

    public default void endVisit(MathBooleanExpressionSymbol node) {

    }

    public default void handle(MathCompareExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathCompareExpressionSymbol node) {

    }

    public default void traverse(MathCompareExpressionSymbol node) {
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        if (node.getRightExpression() != null) handle(node.getRightExpression());
    }

    public default void endVisit(MathCompareExpressionSymbol node) {

    }

    public default void handle(MathConditionalExpressionsSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathConditionalExpressionsSymbol node) {

    }

    public default void traverse(MathConditionalExpressionsSymbol node) {
        if (node.getIfConditionalExpression() != null) handle(node.getIfConditionalExpression());
        for (MathConditionalExpressionSymbol ifElseConditionalExpression : node.getIfElseConditionalExpressions()) {
            handle(ifElseConditionalExpression);
        }
        if (node.getElseConditionalExpression().isPresent())
            handle(node.getElseConditionalExpression().get());
    }

    public default void endVisit(MathConditionalExpressionsSymbol node) {

    }

    public default void handle(MathConditionalExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
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
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathForLoopHeadSymbol node) {

    }

    public default void traverse(MathForLoopHeadSymbol node) {
        handle(node.getMathExpression());
    }

    public default void endVisit(MathForLoopHeadSymbol node) {

    }

    public default void handle(MathForLoopExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathForLoopExpressionSymbol node) {

    }

    public default void traverse(MathForLoopExpressionSymbol node) {
        if (node.getForLoopHead() != null) handle(node.getForLoopHead());
        for (MathExpressionSymbol mathExpressionSymbol : node.getForLoopBody()) {
            handle(mathExpressionSymbol);
        }
    }

    public default void endVisit(MathForLoopExpressionSymbol node) {

    }

    public default void handle(MathNameExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathNameExpressionSymbol node) {

    }

    public default void traverse(MathNameExpressionSymbol node) {

    }

    public default void endVisit(MathNameExpressionSymbol node) {

    }

    public default void handle(MathNumberExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathNumberExpressionSymbol node) {

    }

    public default void traverse(MathNumberExpressionSymbol node) {

    }

    public default void endVisit(MathNumberExpressionSymbol node) {

    }

    public default void handle(MathParenthesisExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathParenthesisExpressionSymbol node) {

    }

    public default void traverse(MathParenthesisExpressionSymbol node) {
        handle(node.getMathExpressionSymbol());
    }

    public default void endVisit(MathParenthesisExpressionSymbol node) {

    }

    public default void handle(MathPreOperatorExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathPreOperatorExpressionSymbol node) {

    }

    public default void traverse(MathPreOperatorExpressionSymbol node) {
        handle(node.getMathExpressionSymbol());
    }

    public default void endVisit(MathPreOperatorExpressionSymbol node) {

    }

    public default void handle(MathValueSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathValueSymbol node) {

    }

    public default void traverse(MathValueSymbol node) {
        if (node.getType() != null) handle(node.getType());
        if (node.getValue() != null) handle(node.getValue());
    }

    public default void endVisit(MathValueSymbol node) {

    }

    public default void handle(MathValueType node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathValueType node) {

    }

    public default void traverse(MathValueType node) {
        for (MathExpressionSymbol dimension : node.getDimensions()) {
            handle(dimension);
        }
    }

    public default void endVisit(MathValueType node) {

    }

    public default void handle(MathMatrixAccessOperatorSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathMatrixAccessOperatorSymbol node) {

    }

    public default void traverse(MathMatrixAccessOperatorSymbol node) {
        if (node.getMathMatrixNameExpressionSymbol() != null) handle(node.getMathMatrixNameExpressionSymbol());
        for (MathMatrixAccessSymbol mathMatrixAccessSymbol : node.getMathMatrixAccessSymbols()) {
            handle(mathMatrixAccessSymbol);
        }
    }

    public default void endVisit(MathMatrixAccessOperatorSymbol node) {

    }

    public default void handle(MathMatrixNameExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathMatrixNameExpressionSymbol node) {

    }

    public default void traverse(MathMatrixNameExpressionSymbol node) {
        if (node.isMathMatrixAccessOperatorSymbolPresent()) handle(node.getMathMatrixAccessOperatorSymbol());
    }

    public default void endVisit(MathMatrixNameExpressionSymbol node) {

    }

    public default void handle(MathMatrixVectorExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathMatrixVectorExpressionSymbol node) {

    }

    public default void traverse(MathMatrixVectorExpressionSymbol node) {
        if (node.getStart() != null) handle(node.getStart());
        if (node.getStep().isPresent())
            handle(node.getStep().get());
        if (node.getEnd() != null) handle(node.getEnd());
    }

    public default void endVisit(MathMatrixVectorExpressionSymbol node) {

    }

    public default void handle(MathMatrixArithmeticValueSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathMatrixArithmeticValueSymbol node) {

    }

    public default void traverse(MathMatrixArithmeticValueSymbol node) {
        for (MathMatrixAccessOperatorSymbol vector : node.getVectors()) {
            handle(vector);
        }
    }

    public default void endVisit(MathMatrixArithmeticValueSymbol node) {

    }

    public default void handle(MathMatrixAccessSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathMatrixAccessSymbol node) {

    }

    public default void traverse(MathMatrixAccessSymbol node) {
        if (node.getMathExpressionSymbol().isPresent())
            handle(node.getMathExpressionSymbol().get());
    }

    public default void endVisit(MathMatrixAccessSymbol node) {

    }

    public default void handle(MathMatrixArithmeticExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void visit(MathMatrixArithmeticExpressionSymbol node) {

    }

    public default void traverse(MathMatrixArithmeticExpressionSymbol node) {
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        if (node.getRightExpression() != null) handle(node.getRightExpression());
    }

    public default void endVisit(MathMatrixArithmeticExpressionSymbol node) {

    }
    

    public Set<MathExpressionSymbol> getVisitedSymbols();

    default boolean shouldContinue(MathExpressionSymbol node) {
        if (node == null || getVisitedSymbols().contains(node))
            return false;
        getVisitedSymbols().add(node);
        return true;
    }
}
