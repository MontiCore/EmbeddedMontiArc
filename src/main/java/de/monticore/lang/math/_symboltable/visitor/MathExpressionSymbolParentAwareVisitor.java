/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.visitor;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;

public interface MathExpressionSymbolParentAwareVisitor extends MathExpressionSymbolVisitor {
    public void pushParent(MathExpressionSymbol parent);

    public void popParent();

    @Override
    public default void handle(MathArithmeticExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathAssignmentExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathBooleanExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathCompareExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathConditionalExpressionsSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathConditionalExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathForLoopHeadSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathForLoopExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathNameExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathNumberExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathParenthesisExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathPreOperatorExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathValueSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathValueType node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathMatrixAccessOperatorSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathMatrixNameExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathMatrixVectorExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathMatrixArithmeticValueSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathMatrixAccessSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    public default void handle(MathMatrixArithmeticExpressionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }
}
