/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable.visitor;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

public interface MathOptExpressionSymbolVisitor extends MathExpressionSymbolVisitor {

    @Override
    default void handle(MathExpressionSymbol node) {
        if (node == null) return;
        else if (node instanceof MathOptimizationStatementSymbol)
            handle((MathOptimizationStatementSymbol) node);
        else if (node instanceof MathOptimizationConditionSymbol)
            handle((MathOptimizationConditionSymbol) node);
        else
            MathExpressionSymbolVisitor.super.handle(node);
    }

    public default void handle(MathOptimizationStatementSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(MathOptimizationStatementSymbol node) {
        if(node.getOptimizationVariables() != null) {
            for (MathValueSymbol optimizationVar : node.getOptimizationVariables()) {
                handle(optimizationVar);
            }
        }
        if(node.getIndependentVariables() != null) {
            for (MathValueSymbol independentVar : node.getIndependentVariables()) {
                handle(independentVar);
            }
        }
        if(node.getConstraints() != null) {
            for (MathOptimizationConditionSymbol constr : node.getConstraints()) {
                handle(constr);
            }
        }
        if(node.getSubjectToExpressions() != null) {
            for (MathExpressionSymbol subjectToExpression : node.getSubjectToExpressions()) {
                handle(subjectToExpression);
            }
        }
        if (node.getStepSizeExpression() != null) handle(node.getStepSizeExpression());
        if (node.getObjectiveValue() != null) handle(node.getObjectiveValue());
        if (node.getObjectiveExpression() != null) handle(node.getObjectiveExpression());

    }

    public default void visit(MathOptimizationStatementSymbol node) {

    }

    public default void endVisit(MathOptimizationStatementSymbol node) {

    }

    public default void handle(MathOptimizationConditionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(MathOptimizationConditionSymbol node) {
        if (node.getLowerBound().isPresent()) handle(node.getLowerBound().get());
        if (node.getUpperBound().isPresent()) handle(node.getUpperBound().get());
        if (node.getBoundedExpression() != null) handle(node.getBoundedExpression());
    }

    public default void visit(MathOptimizationConditionSymbol node) {

    }

    public default void endVisit(MathOptimizationConditionSymbol node) {

    }
}
