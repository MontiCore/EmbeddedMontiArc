/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable.visitor;

import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.CopyMathExpressionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

public class CopyMathOptExpressionSymbol extends CopyMathExpressionSymbol
        implements MathOptExpressionSymbolVisitor {

    public static MathStatementsSymbol copy(MathStatementsSymbol symbol) {
        instance = new CopyMathOptExpressionSymbol();
        return CopyMathExpressionSymbol.copy(symbol);
    }

    public static <T extends MathExpressionSymbol> T copy(T symbol) {
        instance = new CopyMathOptExpressionSymbol();
        return CopyMathExpressionSymbol.copy(symbol);
    }

    @Override
    protected CopyMathExpressionSymbol instantiate() {
        return new CopyMathOptExpressionSymbol();
    }

    @Override
    protected <T extends MathExpressionSymbol> T get(T symbol) {
        MathExpressionSymbol copy = copyMap.get(symbol);
        if (copy != null) return (T) copy;
        if (symbol instanceof MathOptimizationStatementSymbol)
            copy = new MathOptimizationStatementSymbol();
        else if (symbol instanceof MathOptimizationConditionSymbol) {
            MathExpressionSymbol lower = ((MathOptimizationConditionSymbol) symbol).getLowerBound().orElse(null);
            MathExpressionSymbol expr = ((MathOptimizationConditionSymbol) symbol).getBoundedExpression();
            MathExpressionSymbol upper = ((MathOptimizationConditionSymbol) symbol).getUpperBound().orElse(null);
            copy = new MathOptimizationConditionSymbol(lower, expr, upper);
        } else
            copy = super.get(symbol);

        leftToCopy.put(copy, symbol);
        copyMap.put(symbol, copy);
        return (T) copy;
    }

    @Override
    public void endVisit(MathOptimizationStatementSymbol node) {
        MathOptimizationStatementSymbol res = new MathOptimizationStatementSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getOptimizationType() != null)
            res.setOptimizationType(node.getOptimizationType().toString());
        if (node.getOptimizationVariable() != null)
            res.setOptimizationVariable(get(node.getOptimizationVariable()));
        if (node.getObjectiveValue() != null)
            res.setObjectiveValue(get(node.getObjectiveValue()));
        if (node.getObjectiveExpression() != null)
            res.setObjectiveExpression(get(node.getObjectiveExpression()));
        for (MathExpressionSymbol subjectToExpression : node.getSubjectToExpressions())
            res.getSubjectToExpressions().add(get(subjectToExpression));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathOptimizationConditionSymbol node) {
        MathExpressionSymbol lower = node.getLowerBound().orElse(null);
        MathExpressionSymbol expr = node.getBoundedExpression();
        MathExpressionSymbol upper = node.getUpperBound().orElse(null);
        MathOptimizationConditionSymbol res = new MathOptimizationConditionSymbol(lower, expr, upper);
        copyMathExpressionSymbol(res, node);
        copyMap.put(node, res);
    }
}
