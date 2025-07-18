/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable.visitor;

import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.MathValue;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolReplacementVisitor;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

import java.util.ListIterator;
import java.util.Map;
import java.util.function.Function;

public class MathOptExpressionSymbolReplacementVisitor extends MathExpressionSymbolReplacementVisitor
        implements MathOptExpressionSymbolVisitor {

    public static void replace(MathExpressionSymbol mathExpressionSymbol,
                               Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        MathOptExpressionSymbolReplacementVisitor replacementVisitor =
                new MathOptExpressionSymbolReplacementVisitor(replacementMap);
        replacementVisitor.handle(mathExpressionSymbol);
    }

    public static void replace(MathExpressionSymbol mathExpressionSymbol,
                               Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        MathOptExpressionSymbolReplacementVisitor replacementVisitor =
                new MathOptExpressionSymbolReplacementVisitor(replacementFunction);
        replacementVisitor.handle(mathExpressionSymbol);
    }

    public static void replace(MathStatementsSymbol mathStatementsSymbol,
                               Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        MathOptExpressionSymbolReplacementVisitor replacementVisitor =
                new MathOptExpressionSymbolReplacementVisitor(replacementMap);
        replacementVisitor.handle(mathStatementsSymbol);
    }

    public static void replace(MathStatementsSymbol mathStatementsSymbol,
                               Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        MathOptExpressionSymbolReplacementVisitor replacementVisitor =
                new MathOptExpressionSymbolReplacementVisitor(replacementFunction);
        replacementVisitor.handle(mathStatementsSymbol);
    }

    protected MathOptExpressionSymbolReplacementVisitor(Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        super(replacementMap);
    }

    protected MathOptExpressionSymbolReplacementVisitor(Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        super(replacementFunction);
    }

    @Override
    public void visit(MathOptimizationStatementSymbol node) {
        if (node.getOptimizationType() != null)
            node.setOptimizationType(node.getOptimizationType().toString());
        ListIterator<MathValueSymbol> iteratorOptVar = node.getOptimizationVariables().listIterator();
        while (iteratorOptVar.hasNext()){
            MathValueSymbol subjectToExpression = iteratorOptVar.next();
            iteratorOptVar.set(get(subjectToExpression));
        }
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
