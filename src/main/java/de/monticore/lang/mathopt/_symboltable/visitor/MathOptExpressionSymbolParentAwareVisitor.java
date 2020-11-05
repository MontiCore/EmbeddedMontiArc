/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable.visitor;

import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolParentAwareVisitor;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

public interface MathOptExpressionSymbolParentAwareVisitor extends MathExpressionSymbolParentAwareVisitor,
    MathOptExpressionSymbolVisitor {

    @Override
    default void handle(MathOptimizationStatementSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    default void handle(MathOptimizationConditionSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }
}
