/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.math;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;
import de.monticore.symboltable.MutableScope;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class CopyMathSymbol implements MathExpressionSymbolVisitor {
    private Queue<MathExpressionSymbol> childSymbols = new LinkedList<>();

    public static MathStatementsSymbol copy(MathStatementsSymbol symbol) {
        MathStatementsSymbolCopy res = new MathStatementsSymbolCopy(symbol.getName(), symbol.astMathStatements);
        List<MathExpressionSymbol> mathExpressionSymbolsCopy = new LinkedList<>();
        for (MathExpressionSymbol mathExpressionSymbol : symbol.getMathExpressionSymbols()) {
            mathExpressionSymbolsCopy.add(copy(mathExpressionSymbol));
        }
        res.setMathExpressionSymbols(mathExpressionSymbolsCopy);
        return res;
    }

    public static MathExpressionSymbol copy(MathExpressionSymbol symbol) {
        CopyMathSymbol copy = new CopyMathSymbol();
        copy.handle(symbol);
        return copy.childSymbols.poll();
    }

    private void copyMathExpressionSymbol(MathExpressionSymbol newSymbol, MathExpressionSymbol oldSymbol) {
        newSymbol.setID(oldSymbol.getExpressionID());
        newSymbol.setAccessModifier(oldSymbol.getAccessModifier());
        if (oldSymbol.getAstNode().isPresent())
            newSymbol.setAstNode(oldSymbol.getAstNode().get());
        if (oldSymbol.getEnclosingScope() instanceof MutableScope)
            newSymbol.setEnclosingScope((MutableScope) oldSymbol.getEnclosingScope());
        newSymbol.setFullName(oldSymbol.getFullName());
        newSymbol.setPackageName(oldSymbol.getPackageName());
    }

    @Override
    public void endVisit(MathArithmeticExpressionSymbol node) {
        MathArithmeticExpressionSymbol res = new MathArithmeticExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        if(node.getLeftExpression() != null) res.setLeftExpression(childSymbols.poll());
        if(node.getRightExpression() != null) res.setRightExpression(childSymbols.poll());
        res.setMathOperator(node.getMathOperator());
        res.setOperator(node.getOperator());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathAssignmentExpressionSymbol node) {
        MathAssignmentExpressionSymbol res = new MathAssignmentExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setNameOfMathValue(node.getNameOfMathValue());
        if(node.getExpressionSymbol() != null) res.setExpressionSymbol(childSymbols.poll());
        res.setAssignmentOperator(node.getAssignmentOperator());
        res.setMathMatrixAccessOperatorSymbol(node.getMathMatrixAccessOperatorSymbol());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathBooleanExpressionSymbol node) {
        MathBooleanExpressionSymbol res = new MathBooleanExpressionSymbol(Boolean.valueOf(node.getTextualRepresentation()));
        copyMathExpressionSymbol(res, node);
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathCompareExpressionSymbol node) {
        MathCompareExpressionSymbol res = new MathCompareExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setCompareOperator(node.getCompareOperator());
        if(node.getLeftExpression() != null) res.setLeftExpression(childSymbols.poll());
        if(node.getRightExpression() != null) res.setRightExpression(childSymbols.poll());
        res.setOperator(node.getOperator());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathConditionalExpressionsSymbol node) {
        MathConditionalExpressionsSymbol res = new MathConditionalExpressionsSymbol();
        copyMathExpressionSymbol(res, node);
        res.setIfConditionalExpression((MathConditionalExpressionSymbol) childSymbols.poll());
        res.setIfElseConditionalExpressions(new LinkedList<>());
        for (int i = 0; i < node.getIfElseConditionalExpressions().size(); i++) {
            res.getIfElseConditionalExpressions().add((MathConditionalExpressionSymbol) childSymbols.poll());
        }
        if (node.getElseConditionalExpression().isPresent()) res.setElseConditionalExpression((MathConditionalExpressionSymbol) childSymbols.poll());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathConditionalExpressionSymbol node) {
        MathConditionalExpressionSymbol res = new MathConditionalExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setCondition(childSymbols.poll());
        res.setBodyExpressions(new LinkedList<>());
        for (int i = 0; i < node.getBodyExpressions().size(); i++) {
            res.getBodyExpressions().add(childSymbols.poll());
        }
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathForLoopHeadSymbol node) {
        MathForLoopHeadSymbol res = new MathForLoopHeadSymbol();
        copyMathExpressionSymbol(res, node);
        res.setNameLoopVariable(node.getNameLoopVariable());
        if(node.getMathExpression() != null) res.setMathExpression(childSymbols.poll());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathForLoopExpressionSymbol node) {
        MathForLoopExpressionSymbol res = new MathForLoopExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setForLoopHead((MathForLoopHeadSymbol) childSymbols.poll());
        for (int i = 0; i < node.getForLoopBody().size(); i++) {
            res.addForLoopBody(childSymbols.poll());
        }
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathNameExpressionSymbol node) {
        MathNameExpressionSymbol res = new MathNameExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setNameToResolveValue(node.getNameToResolveValue());
        res.setNameToAccess(node.getNameToAccess());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathNumberExpressionSymbol node) {
        MathNumberExpressionSymbol res = new MathNumberExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setValue(node.getValue());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathParenthesisExpressionSymbol node) {
        MathParenthesisExpressionSymbol res = new MathParenthesisExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setMathExpressionSymbol(childSymbols.poll());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathPreOperatorExpressionSymbol node) {
        MathPreOperatorExpressionSymbol res = new MathPreOperatorExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setMathExpressionSymbol(childSymbols.poll());
        res.setOperator(node.getOperator());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathValueSymbol node) {
        MathValueSymbol res = new MathValueSymbol(node.getName());
        copyMathExpressionSymbol(res, node);
        res.setMatrixProperties(node.getMatrixProperties());
        if (node.getType() != null) res.setType((MathValueType) childSymbols.poll());
        if (node.getValue() != null) res.setValue(childSymbols.poll());
        childSymbols.add(res);
    }

    @Override
    public void endVisit(MathValueType node) {
        MathValueType res = new MathValueType();
        copyMathExpressionSymbol(res, node);
        res.setProperties(node.getProperties());
        res.setType(node.getType());
        res.setTypeRef(node.getTypeRef());
        res.setDimensions(new LinkedList<>());
        for (int i = 0; i < node.getDimensions().size(); i++) {
            res.getDimensions().add(childSymbols.poll());
        }
        childSymbols.add(res);
    }
}
