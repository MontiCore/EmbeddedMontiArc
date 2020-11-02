/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialGuessSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialValueSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;

public interface EMAMMathExpressionSymbolVisitor extends MathExpressionSymbolVisitor {

    @Override
    public default void handle(MathExpressionSymbol node) {
        if (node == null) return;
        else if (node instanceof EMAMEquationSymbol)
            handle((EMAMEquationSymbol) node);
        else if (node instanceof EMAMInitialGuessSymbol)
            handle((EMAMInitialGuessSymbol) node);
        else if (node instanceof EMAMInitialValueSymbol)
            handle((EMAMInitialValueSymbol) node);
        else
            MathExpressionSymbolVisitor.super.handle(node);
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
}
