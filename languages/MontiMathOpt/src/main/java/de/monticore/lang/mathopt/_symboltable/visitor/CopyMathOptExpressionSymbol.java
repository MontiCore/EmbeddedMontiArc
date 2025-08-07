/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable.visitor;

import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.visitor.CopyMathExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathStatementsSymbolCopy;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

import java.util.LinkedList;
import java.util.List;

public class CopyMathOptExpressionSymbol extends CopyMathExpressionSymbol
        implements MathOptExpressionSymbolVisitor {

    public static MathStatementsSymbol copy(MathStatementsSymbol symbol) {
        MathStatementsSymbolCopy res = new MathStatementsSymbolCopy(symbol.getName(), symbol.astMathStatements);
        if (symbol.getAstNode().isPresent()) res.setAstNode(symbol.getAstNode().get());
        res.setAccessModifier(symbol.getAccessModifier());
        res.setPackageName(symbol.getPackageName());
        res.setFullName(symbol.getFullName());
        List<MathExpressionSymbol> mathExpressionSymbolsCopy = new LinkedList<>();
        for (MathExpressionSymbol mathExpressionSymbol : symbol.getMathExpressionSymbols()) {
            mathExpressionSymbolsCopy.add(copy(mathExpressionSymbol));
        }
        res.setMathExpressionSymbols(mathExpressionSymbolsCopy);
        return res;
    }

    public static <T extends MathExpressionSymbol> T copy(T symbol) {
        CopyMathOptExpressionSymbol copy = new CopyMathOptExpressionSymbol();
        copy.handle(symbol);
        T res = copy.get(symbol);
        return res;
    }

    @Override
    protected <T extends MathExpressionSymbol> T get(T symbol) {
        MathExpressionSymbol copy = copyMap.get(symbol);
        if (copy != null) return (T) copy;
        if (symbol instanceof MathOptimizationStatementSymbol)
            copy = new MathOptimizationStatementSymbol();
        else if (symbol instanceof MathOptimizationConditionSymbol)
            copy = new MathOptimizationConditionSymbol(null, "", null);
        else
            return super.get(symbol);

        copyMap.put(symbol, copy);
        return (T) copy;
    }

    @Override
    public void endVisit(MathOptimizationStatementSymbol node) {
        MathOptimizationStatementSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getOptimizationType() != null)
            res.setOptimizationType(node.getOptimizationType().toString());
        for (MathValueSymbol optimizationVariable : node.getOptimizationVariables())
            res.getOptimizationVariables().add(get(optimizationVariable));
        for (MathValueSymbol independentVariable : node.getIndependentVariables())
            res.getIndependentVariables().add(get(independentVariable));
        for (MathOptimizationConditionSymbol constr : node.getConstraints())
            res.getConstraints().add(get(constr));
        for (MathExpressionSymbol subjectToExpression : node.getSubjectToExpressions())
            res.getSubjectToExpressions().add(get(subjectToExpression));

        if(node.getStepSizeExpression() != null)
            res.setStepSizeExpression(get(node.getStepSizeExpression()));

        if (node.getObjectiveValue() != null)
            res.setObjectiveValue(get(node.getObjectiveValue()));
        if (node.getObjectiveExpression() != null)
            res.setObjectiveExpression(get(node.getObjectiveExpression()));

        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathOptimizationConditionSymbol node) {
        MathOptimizationConditionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);

        if(node.getOperator() != null)
            res.setOperator(node.getOperator());
        if(node.getLeft() != null)
            res.setLeft(get(node.getLeft()));
        if(node.getRight() != null)
            res.setRight(get(node.getRight()));
        res.setSimpleCondition(node.isSimpleCondition());

        if (node.getLowerBound().isPresent())
            res.setLowerBound(get(node.getLowerBound().get()));
        if (node.getBoundedExpression() != null)
            res.setBoundedExpression(get(node.getBoundedExpression()));
        if (node.getUpperBound().isPresent())
            res.setUpperBound(get(node.getUpperBound().get()));
        copyMap.put(node, res);
    }
}
