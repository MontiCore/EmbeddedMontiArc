/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.visitor;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;

import java.util.Stack;

public interface MathExpressionSymbolParentAwareVisitor extends MathExpressionSymbolVisitor {

    Stack<MathExpressionSymbol> getParents();

    default void pushParent(MathExpressionSymbol parent) {
        getParents().push(parent);
    }

    default void popParent() {
        getParents().pop();
    }

    @Override
    public default void handle(MathArithmeticExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathAssignmentExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathBooleanExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathCompareExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathConditionalExpressionsSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathConditionalExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathForLoopHeadSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathForLoopExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathNameExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathNumberExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathParenthesisExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathPreOperatorExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathValueSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathValueType node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathMatrixAccessOperatorSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathMatrixNameExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathMatrixVectorExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathMatrixArithmeticValueSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathMatrixAccessSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    public default void handle(MathMatrixArithmeticExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }
}
