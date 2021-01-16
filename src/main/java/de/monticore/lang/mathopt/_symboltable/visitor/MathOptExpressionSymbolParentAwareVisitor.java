/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable.visitor;

import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolParentAwareVisitor;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

public interface MathOptExpressionSymbolParentAwareVisitor extends MathExpressionSymbolParentAwareVisitor,
        MathOptExpressionSymbolVisitor {

    @Override
    default void handle(MathOptimizationStatementSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }

    @Override
    default void handle(MathOptimizationConditionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }
}
