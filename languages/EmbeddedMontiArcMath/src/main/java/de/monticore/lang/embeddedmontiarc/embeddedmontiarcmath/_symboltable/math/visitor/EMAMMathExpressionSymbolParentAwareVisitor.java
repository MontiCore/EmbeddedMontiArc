/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.mathopt._symboltable.visitor.MathOptExpressionSymbolParentAwareVisitor;

public interface EMAMMathExpressionSymbolParentAwareVisitor extends MathOptExpressionSymbolParentAwareVisitor,
    EMAMMathExpressionSymbolVisitor {

    @Override
    default void handle(EMAMSpecificationSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    default void handle(EMAMSymbolicVariableSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    default void handle(EMAMEquationSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    default void handle(EMAMInitialGuessSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    default void handle(EMAMInitialValueSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    default void handle(MathStringExpression node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    default void handle(MathChainedExpression node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }
}
