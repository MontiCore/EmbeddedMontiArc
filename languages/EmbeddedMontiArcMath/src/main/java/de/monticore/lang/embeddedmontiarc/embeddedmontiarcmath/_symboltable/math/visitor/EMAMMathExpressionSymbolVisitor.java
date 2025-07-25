/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.mathopt._symboltable.visitor.MathOptExpressionSymbolVisitor;

public interface EMAMMathExpressionSymbolVisitor extends MathOptExpressionSymbolVisitor {

    @Override
    public default void handle(MathExpressionSymbol node) {
        if (node == null) return;
        else if (node instanceof EMAMSpecificationSymbol)
            handle((EMAMSpecificationSymbol) node);
        else if (node instanceof EMAMEquationSymbol)
            handle((EMAMEquationSymbol) node);
        else if (node instanceof EMAMInitialGuessSymbol)
            handle((EMAMInitialGuessSymbol) node);
        else if (node instanceof EMAMInitialValueSymbol)
            handle((EMAMInitialValueSymbol) node);
        else if (node instanceof MathStringExpression)
            handle((MathStringExpression) node);
        else if (node instanceof MathChainedExpression)
            handle((MathChainedExpression) node);
        else
            MathOptExpressionSymbolVisitor.super.handle(node);
    }

    public default void handle(EMAMSpecificationSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(EMAMSpecificationSymbol node) {
        for (EMAMSymbolicVariableSymbol variable : node.getVariables())
            handle(variable);
        for (EMAMInitialValueSymbol initialValue : node.getInitialValues())
            handle(initialValue);
        for (EMAMInitialGuessSymbol initialGuess : node.getInitialGuesses())
            handle(initialGuess);
        for (EMAMEquationSymbol equation : node.getEquations())
            handle(equation);
    }

    public default void visit(EMAMSpecificationSymbol node) {

    }

    public default void endVisit(EMAMSpecificationSymbol node) {

    }


    public default void handle(EMAMSymbolicVariableSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(EMAMSymbolicVariableSymbol node) {
        if (node.getType() != null) handle(node.getType());
    }

    public default void visit(EMAMSymbolicVariableSymbol node) {

    }

    public default void endVisit(EMAMSymbolicVariableSymbol node) {

    }

    public default void handle(EMAMEquationSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(EMAMEquationSymbol node) {
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        if (node.getRightExpression() != null) handle(node.getRightExpression());
    }

    public default void visit(EMAMEquationSymbol node) {

    }

    public default void endVisit(EMAMEquationSymbol node) {

    }


    public default void handle(EMAMInitialGuessSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(EMAMInitialGuessSymbol node) {
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            handle(node.getMathMatrixAccessOperatorSymbol());
        if (node.getValue() != null) handle(node.getValue());
    }

    public default void visit(EMAMInitialGuessSymbol node) {

    }

    public default void endVisit(EMAMInitialGuessSymbol node) {

    }


    public default void handle(EMAMInitialValueSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(EMAMInitialValueSymbol node) {
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            handle(node.getMathMatrixAccessOperatorSymbol());
        if (node.getValue() != null) handle(node.getValue());
    }

    public default void visit(EMAMInitialValueSymbol node) {

    }

    public default void endVisit(EMAMInitialValueSymbol node) {

    }


    public default void handle(MathStringExpression node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(MathStringExpression node) {
        for (MathMatrixAccessSymbol previousExpressionSymbol : node.getPreviousExpressionSymbols())
            handle(previousExpressionSymbol);
    }

    public default void visit(MathStringExpression node) {

    }

    public default void endVisit(MathStringExpression node) {

    }


    public default void handle(MathChainedExpression node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(MathChainedExpression node) {
        if (node.getFirstExpressionSymbol() != null)
            handle(node.getFirstExpressionSymbol());
        if (node.getSecondExpressionSymbol() != null)
            handle(node.getSecondExpressionSymbol());
    }

    public default void visit(MathChainedExpression node) {

    }

    public default void endVisit(MathChainedExpression node) {

    }
}
