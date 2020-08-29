/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.copy;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;
import de.monticore.symboltable.MutableScope;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Stack;

public class CopyMathExpressionSymbol implements MathExpressionSymbolVisitor {
    private Stack<Queue<MathExpressionSymbol>> childSymbols;

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
        CopyMathExpressionSymbol copy = new CopyMathExpressionSymbol();
        copy.handle(symbol);
        return copy.childSymbols.pop().poll();
    }

    public CopyMathExpressionSymbol() {
        childSymbols = new Stack<>();
        childSymbols.add(new LinkedList<>());
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
    public void visit(MathArithmeticExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathAssignmentExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathBooleanExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathCompareExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathConditionalExpressionsSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathConditionalExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathForLoopHeadSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathForLoopExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathNameExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathNumberExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathParenthesisExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathPreOperatorExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathValueSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void visit(MathValueType node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override public void visit(MathMatrixAccessOperatorSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override public void visit(MathMatrixNameExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override public void visit(MathMatrixVectorExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override public void visit(MathMatrixArithmeticValueSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override public void visit(MathMatrixAccessSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override public void visit(MathMatrixArithmeticExpressionSymbol node) {
        childSymbols.push(new LinkedList<>());
    }

    @Override
    public void endVisit(MathArithmeticExpressionSymbol node) {
        MathArithmeticExpressionSymbol res = new MathArithmeticExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getLeftExpression() != null) res.setLeftExpression(childSymbols.peek().poll());
        if (node.getRightExpression() != null) res.setRightExpression(childSymbols.peek().poll());
        res.setMathOperator(node.getMathOperator());
        res.setOperator(node.getOperator());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathAssignmentExpressionSymbol node) {
        MathAssignmentExpressionSymbol res = new MathAssignmentExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setNameOfMathValue(node.getNameOfMathValue());
        if (node.getMathMatrixAccessOperatorSymbol() != null) res.setMathMatrixAccessOperatorSymbol((MathMatrixAccessOperatorSymbol) childSymbols.peek().poll());
        if (node.getExpressionSymbol() != null) res.setExpressionSymbol(childSymbols.peek().poll());
        res.setAssignmentOperator(node.getAssignmentOperator());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathBooleanExpressionSymbol node) {
        MathBooleanExpressionSymbol res = new MathBooleanExpressionSymbol(Boolean.valueOf(node.getTextualRepresentation()));
        copyMathExpressionSymbol(res, node);
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathCompareExpressionSymbol node) {
        MathCompareExpressionSymbol res = new MathCompareExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setCompareOperator(node.getCompareOperator());
        if (node.getLeftExpression() != null) res.setLeftExpression(childSymbols.peek().poll());
        if (node.getRightExpression() != null) res.setRightExpression(childSymbols.peek().poll());
        res.setOperator(node.getOperator());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathConditionalExpressionsSymbol node) {
        MathConditionalExpressionsSymbol res = new MathConditionalExpressionsSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getIfConditionalExpression() != null) res.setIfConditionalExpression((MathConditionalExpressionSymbol) childSymbols.peek().poll());
        res.setIfElseConditionalExpressions(new LinkedList<>());
        for (int i = 0; i < node.getIfElseConditionalExpressions().size(); i++) {
            res.getIfElseConditionalExpressions().add((MathConditionalExpressionSymbol) childSymbols.peek().poll());
        }
        if (node.getElseConditionalExpression().isPresent())
            res.setElseConditionalExpression((MathConditionalExpressionSymbol) childSymbols.peek().poll());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathConditionalExpressionSymbol node) {
        MathConditionalExpressionSymbol res = new MathConditionalExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getCondition() != null) res.setCondition(childSymbols.peek().poll());
        res.setBodyExpressions(new LinkedList<>());
        for (int i = 0; i < node.getBodyExpressions().size(); i++) {
            res.getBodyExpressions().add(childSymbols.peek().poll());
        }
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathForLoopHeadSymbol node) {
        MathForLoopHeadSymbol res = new MathForLoopHeadSymbol();
        copyMathExpressionSymbol(res, node);
        res.setNameLoopVariable(node.getNameLoopVariable());
        if (node.getMathExpression() != null) res.setMathExpression(childSymbols.peek().poll());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathForLoopExpressionSymbol node) {
        MathForLoopExpressionSymbol res = new MathForLoopExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getForLoopHead() != null) res.setForLoopHead((MathForLoopHeadSymbol) childSymbols.peek().poll());
        for (int i = 0; i < node.getForLoopBody().size(); i++) {
            res.addForLoopBody(childSymbols.peek().poll());
        }
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathNameExpressionSymbol node) {
        MathNameExpressionSymbol res = new MathNameExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setNameToResolveValue(node.getNameToResolveValue());
        res.setNameToAccess(node.getNameToAccess());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathNumberExpressionSymbol node) {
        MathNumberExpressionSymbol res = new MathNumberExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setValue(node.getValue());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathParenthesisExpressionSymbol node) {
        MathParenthesisExpressionSymbol res = new MathParenthesisExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getMathExpressionSymbol() != null) res.setMathExpressionSymbol(childSymbols.peek().poll());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathPreOperatorExpressionSymbol node) {
        MathPreOperatorExpressionSymbol res = new MathPreOperatorExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getMathExpressionSymbol() != null) res.setMathExpressionSymbol(childSymbols.peek().poll());
        res.setOperator(node.getOperator());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override
    public void endVisit(MathValueSymbol node) {
        MathValueSymbol res = new MathValueSymbol(node.getName());
        copyMathExpressionSymbol(res, node);
        res.setMatrixProperties(node.getMatrixProperties());
        if (node.getType() != null) res.setType((MathValueType) childSymbols.peek().poll());
        if (node.getValue() != null) res.setValue(childSymbols.peek().poll());
        childSymbols.pop();
        childSymbols.peek().add(res);
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
            res.getDimensions().add(childSymbols.peek().poll());
        }
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override public void endVisit(MathMatrixAccessOperatorSymbol node) {
        MathMatrixAccessOperatorSymbol res = new MathMatrixAccessOperatorSymbol();
        copyMathExpressionSymbol(res, node);
        res.setAccessStartSymbol(node.getAccessStartSymbol());
        res.setAccessEndSymbol(node.getAccessEndSymbol());
        if (node.getMathMatrixNameExpressionSymbol() != null) res.setMathMatrixNameExpressionSymbol((MathMatrixNameExpressionSymbol) childSymbols.peek().poll());
        if (node.getMathMatrixNameExpressionSymbol() != null) res.getMathMatrixNameExpressionSymbol().setMathMatrixAccessOperatorSymbol(res);
        res.setMathMatrixAccessSymbols(new LinkedList<>());
        for (int i = 0; i < node.getMathMatrixAccessSymbols().size(); i++) {
            res.getMathMatrixAccessSymbols().add((MathMatrixAccessSymbol) childSymbols.peek().poll());
        }
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override public void endVisit(MathMatrixNameExpressionSymbol node) {
        MathMatrixNameExpressionSymbol res = new MathMatrixNameExpressionSymbol(node.getNameToAccess());
        copyMathExpressionSymbol(res, node);
        if (node.isASTMathMatrixNamePresent())
            res.setAstMathMatrixNameExpression(node.getAstMathMatrixNameExpression());
        res.setNameToAccess(node.getNameToAccess());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override public void endVisit(MathMatrixVectorExpressionSymbol node) {
        MathMatrixVectorExpressionSymbol res = new MathMatrixVectorExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getStart() != null) res.setStart(childSymbols.peek().poll());
        if (node.getStep().isPresent())
            res.setStep(childSymbols.peek().poll());
        if (node.getEnd() != null) res.setEnd(childSymbols.peek().poll());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override public void endVisit(MathMatrixArithmeticValueSymbol node) {
        MathMatrixArithmeticValueSymbol res = new MathMatrixArithmeticValueSymbol();
        copyMathExpressionSymbol(res, node);
        res.setMatrixProperties(node.getMatrixProperties());
        res.setVectors(new LinkedList<>());
        for (int i = 0; i < node.getVectors().size(); i++) {
            res.getVectors().add((MathMatrixAccessOperatorSymbol) childSymbols.peek().poll());
        }
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override public void endVisit(MathMatrixAccessSymbol node) {
        MathMatrixAccessSymbol res = new MathMatrixAccessSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getMathExpressionSymbol().isPresent())
            res.setMathExpressionSymbol(childSymbols.peek().poll());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }

    @Override public void endVisit(MathMatrixArithmeticExpressionSymbol node) {
        MathMatrixArithmeticExpressionSymbol res = new MathMatrixArithmeticExpressionSymbol();
        copyMathExpressionSymbol(res, node);
        res.setOperator(node.getOperator());
        res.setMathOperator(node.getMathOperator());
        if (node.getLeftExpression() != null) res.setLeftExpression(childSymbols.peek().poll());
        if (node.getRightExpression() != null) res.setRightExpression(childSymbols.peek().poll());
        childSymbols.pop();
        childSymbols.peek().add(res);
    }
}
