/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable.visitor;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolReplacementVisitor;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

import java.util.Iterator;
import java.util.ListIterator;
import java.util.Map;
import java.util.function.Function;

public class MathOptExpressionSymbolReplacementVisitor extends MathExpressionSymbolReplacementVisitor
        implements MathOptExpressionSymbolVisitor {

    public MathOptExpressionSymbolReplacementVisitor(Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        super(replacementMap);
    }

    public MathOptExpressionSymbolReplacementVisitor(Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        super(replacementFunction);
    }

    @Override
    public void visit(MathOptimizationStatementSymbol node) {
        if (node.getOptimizationType() != null)
            node.setOptimizationType(node.getOptimizationType().toString());
        if (node.getOptimizationVariable() != null)
            node.setOptimizationVariable(get(node.getOptimizationVariable()));
        if (node.getObjectiveValue() != null)
            node.setObjectiveValue(get(node.getObjectiveValue()));
        if (node.getObjectiveExpression() != null)
            node.setObjectiveExpression(get(node.getObjectiveExpression()));
        ListIterator<MathExpressionSymbol> iterator = node.getSubjectToExpressions().listIterator();
        while (iterator.hasNext()){
            MathExpressionSymbol subjectToExpression = iterator.next();
            iterator.set(get(subjectToExpression));
        }
    }

    @Override
    public void visit(MathOptimizationConditionSymbol node) {
        if (node.getLowerBound().isPresent())
            node.setLowerBound(get(node.getLowerBound().get()));
        if (node.getBoundedExpression() != null)
            node.setBoundedExpression(get(node.getBoundedExpression()));
        if (node.getUpperBound().isPresent())
            node.setUpperBound(get(node.getUpperBound().get()));
    }
}
