/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.visitor;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;

import java.util.HashSet;
import java.util.ListIterator;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;

public class MathExpressionSymbolReplacementVisitor implements MathExpressionSymbolVisitor {

    private Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction;
    private Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap;

    public static void replace(MathExpressionSymbol mathExpressionSymbol,
                               Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        MathExpressionSymbolReplacementVisitor replacementVisitor =
                new MathExpressionSymbolReplacementVisitor(replacementMap);
        replacementVisitor.handle(mathExpressionSymbol);
        replacementVisitor.handle(mathExpressionSymbol);
    }

    public static void replace(MathExpressionSymbol mathExpressionSymbol,
                               Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        MathExpressionSymbolReplacementVisitor replacementVisitor =
                new MathExpressionSymbolReplacementVisitor(replacementFunction);
        replacementVisitor.handle(mathExpressionSymbol);
        replacementVisitor.handle(mathExpressionSymbol);
    }

    public static void replace(MathStatementsSymbol mathStatementsSymbol,
                               Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        MathExpressionSymbolReplacementVisitor replacementVisitor =
                new MathExpressionSymbolReplacementVisitor(replacementMap);
        replacementVisitor.handle(mathStatementsSymbol);
        replacementVisitor.handle(mathStatementsSymbol);
    }

    public static void replace(MathStatementsSymbol mathStatementsSymbol,
                               Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        MathExpressionSymbolReplacementVisitor replacementVisitor =
                new MathExpressionSymbolReplacementVisitor(replacementFunction);
        replacementVisitor.handle(mathStatementsSymbol);
        replacementVisitor.handle(mathStatementsSymbol);
    }

    protected MathExpressionSymbolReplacementVisitor(Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        this.replacementMap = replacementMap;
    }

    protected MathExpressionSymbolReplacementVisitor(Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        this.replacementFunction = replacementFunction;
    }

    protected <T extends MathExpressionSymbol> T get(T symbol) {
        if (replacementMap != null && replacementMap.keySet().contains(symbol))
            return (T) replacementMap.get(symbol);
        if (replacementFunction != null) {
            MathExpressionSymbol apply = replacementFunction.apply(symbol);
            if (apply != null) return (T) apply;
        }
        return symbol;
    }

    public void handle(MathStatementsSymbol mathStatementsSymbol) {
        ListIterator<MathExpressionSymbol> iterator = mathStatementsSymbol.getMathExpressionSymbols().listIterator();
        while (iterator.hasNext()) {
            MathExpressionSymbol expression = iterator.next();
            handle(expression);
            iterator.set(get(expression));
        }
    }

    @Override
    public void visit(MathArithmeticExpressionSymbol node) {
        if (node.getLeftExpression() != null) node.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) node.setRightExpression(get(node.getRightExpression()));
    }

    @Override
    public void visit(MathAssignmentExpressionSymbol node) {
        if (node.getMathMatrixAccessOperatorSymbol() != null)
            node.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getExpressionSymbol() != null)
            node.setExpressionSymbol(get(node.getExpressionSymbol()));
    }

    @Override
    public void visit(MathBooleanExpressionSymbol node) {

    }

    @Override
    public void visit(MathCompareExpressionSymbol node) {
        if (node.getLeftExpression() != null) node.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) node.setRightExpression(get(node.getRightExpression()));
    }

    @Override
    public void visit(MathConditionalExpressionsSymbol node) {
        if (node.getIfConditionalExpression() != null)
            node.setIfConditionalExpression((MathConditionalExpressionSymbol) get(node.getIfConditionalExpression()));
        ListIterator<MathConditionalExpressionSymbol> iterator = node.getIfElseConditionalExpressions().listIterator();
        while(iterator.hasNext()) {
            MathConditionalExpressionSymbol next = iterator.next();
            iterator.set((MathConditionalExpressionSymbol) get(next));
        }
        if (node.getElseConditionalExpression().isPresent())
            node.setElseConditionalExpression((MathConditionalExpressionSymbol) get(node.getElseConditionalExpression().get()));
    }

    @Override
    public void visit(MathConditionalExpressionSymbol node) {
        if (node.getCondition().isPresent()) node.setCondition(get(node.getCondition().get()));
        ListIterator<MathExpressionSymbol> iterator = node.getBodyExpressions().listIterator();
        while(iterator.hasNext()) {
            MathExpressionSymbol next = iterator.next();
            iterator.set(get(next));
        }
    }

    @Override
    public void visit(MathForLoopHeadSymbol node) {
        if (node.getMathExpression() != null) node.setMathExpression(get(node.getMathExpression()));
    }

    @Override
    public void visit(MathForLoopExpressionSymbol node) {
        if (node.getForLoopHead() != null) node.setForLoopHead((MathForLoopHeadSymbol) get(node.getForLoopHead()));
        ListIterator<MathExpressionSymbol> iterator = node.getForLoopBody().listIterator();
        while(iterator.hasNext()) {
            MathExpressionSymbol next = iterator.next();
            iterator.set(get(next));
        }
    }

    @Override
    public void visit(MathNameExpressionSymbol node) {
    }

    @Override
    public void visit(MathNumberExpressionSymbol node) {
    }

    @Override
    public void visit(MathParenthesisExpressionSymbol node) {
        if (node.getMathExpressionSymbol() != null) node.setMathExpressionSymbol(get(node.getMathExpressionSymbol()));
    }

    @Override
    public void visit(MathPreOperatorExpressionSymbol node) {
        if (node.getMathExpressionSymbol() != null) node.setMathExpressionSymbol(get(node.getMathExpressionSymbol()));
    }

    @Override
    public void visit(MathValueSymbol node) {
        if (node.getType() != null) node.setType((MathValueType) get(node.getType()));
        if (node.getValue() != null) node.setValue(get(node.getValue()));
    }

    @Override
    public void visit(MathValueType node) {
        ListIterator<MathExpressionSymbol> iterator = node.getDimensions().listIterator();
        while(iterator.hasNext()) {
            MathExpressionSymbol next = iterator.next();
            iterator.set(get(next));
        }
    }

    @Override public void visit(MathMatrixAccessOperatorSymbol node) {
        if (node.getMathMatrixNameExpressionSymbol() != null)
            node.setMathMatrixNameExpressionSymbol(get(node.getMathMatrixNameExpressionSymbol()));
        ListIterator<MathMatrixAccessSymbol> iterator = node.getMathMatrixAccessSymbols().listIterator();
        while(iterator.hasNext()) {
            MathExpressionSymbol next = iterator.next();
            iterator.set((MathMatrixAccessSymbol) get(next));
        }
    }

    @Override public void visit(MathMatrixNameExpressionSymbol node) {
        if (node.isASTMathMatrixNamePresent())
            node.setAstMathMatrixNameExpression(node.getAstMathMatrixNameExpression());
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            node.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
    }

    @Override public void visit(MathMatrixVectorExpressionSymbol node) {
        if (node.getStart() != null) node.setStart(get(node.getStart()));
        if (node.getStep().isPresent())
            node.setStep(get(node.getStep().get()));
        if (node.getEnd() != null) node.setEnd(get(node.getEnd()));
    }

    @Override public void visit(MathMatrixArithmeticValueSymbol node) {
        ListIterator<MathMatrixAccessOperatorSymbol> iterator = node.getVectors().listIterator();
        while(iterator.hasNext()) {
            MathExpressionSymbol next = iterator.next();
            iterator.set((MathMatrixAccessOperatorSymbol) get(next));
        }
    }

    @Override public void visit(MathMatrixAccessSymbol node) {
        if (node.getMathExpressionSymbol().isPresent())
            node.setMathExpressionSymbol(get(node.getMathExpressionSymbol().get()));
    }

    @Override public void visit(MathMatrixArithmeticExpressionSymbol node) {
        if (node.getLeftExpression() != null) node.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) node.setRightExpression(get(node.getRightExpression()));
    }

    protected Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

    @Override
    public Set<MathExpressionSymbol> getVisitedSymbols() {
        return visitedSymbols;
    }
}
