/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.visitor;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.symboltable.MutableScope;

import java.util.*;

public class CopyMathExpressionSymbol implements MathExpressionSymbolVisitor {

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
        CopyMathExpressionSymbol copy = new CopyMathExpressionSymbol();
        copy.handle(symbol);
        T res = copy.get(symbol);
        return res;
    }

    protected Map<MathExpressionSymbol, MathExpressionSymbol> copyMap;

    public CopyMathExpressionSymbol() {
        copyMap = new HashMap<>();
    }

    protected <T extends MathExpressionSymbol> T get(T symbol) {
        MathExpressionSymbol copy = copyMap.get(symbol);
        if (copy != null) return (T) copy;
        if (symbol instanceof MathArithmeticExpressionSymbol)
            copy = new MathArithmeticExpressionSymbol();
        else if (symbol instanceof MathAssignmentExpressionSymbol)
            copy = new MathAssignmentExpressionSymbol();
        else if (symbol instanceof MathCompareExpressionSymbol)
            copy = new MathCompareExpressionSymbol();
        else if (symbol instanceof MathConditionalExpressionsSymbol)
            copy = new MathConditionalExpressionsSymbol();
        else if (symbol instanceof MathConditionalExpressionSymbol)
            copy = new MathConditionalExpressionSymbol();
        else if (symbol instanceof MathForLoopHeadSymbol)
            copy = new MathForLoopHeadSymbol();
        else if (symbol instanceof MathForLoopExpressionSymbol)
            copy = new MathForLoopExpressionSymbol();
        else if (symbol instanceof MathParenthesisExpressionSymbol)
            copy = new MathParenthesisExpressionSymbol();
        else if (symbol instanceof MathPreOperatorExpressionSymbol)
            copy = new MathPreOperatorExpressionSymbol();
        else if (symbol instanceof MathValueType)
            copy = new MathValueType();
        else if (symbol instanceof MathNameExpressionSymbol)
            copy = new MathNameExpressionSymbol();
        else if (symbol instanceof MathValueSymbol)
            copy = new MathValueSymbol(symbol.getName());
        else if (symbol instanceof MathNumberExpressionSymbol)
            copy = new MathNumberExpressionSymbol();
        else if (symbol instanceof MathBooleanExpressionSymbol)
            copy = new MathBooleanExpressionSymbol(Boolean.parseBoolean(symbol.getTextualRepresentation()));
        else if (symbol instanceof MathMatrixAccessOperatorSymbol)
            copy = new MathMatrixAccessOperatorSymbol();
        else if (symbol instanceof MathMatrixNameExpressionSymbol)
            copy = new MathMatrixNameExpressionSymbol(((MathMatrixNameExpressionSymbol) symbol).getNameToAccess());
        else if (symbol instanceof MathMatrixVectorExpressionSymbol)
            copy = new MathMatrixVectorExpressionSymbol();
        else if (symbol instanceof MathMatrixArithmeticValueSymbol)
            copy = new MathMatrixArithmeticValueSymbol();
        else if (symbol instanceof MathMatrixAccessSymbol)
            copy = new MathMatrixAccessSymbol();
        else if (symbol instanceof MathMatrixArithmeticExpressionSymbol)
            copy = new MathMatrixArithmeticExpressionSymbol();

        copyMap.put(symbol, copy);
        return (T) copy;
    }

    protected void copyMathExpressionSymbol(MathExpressionSymbol newSymbol, MathExpressionSymbol oldSymbol) {
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
        if (node.getLeftExpression() != null) res.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) res.setRightExpression(get(node.getRightExpression()));
        res.setMathOperator(node.getMathOperator());
        res.setOperator(node.getOperator());
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathAssignmentExpressionSymbol node) {
        MathAssignmentExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setNameOfMathValue(node.getNameOfMathValue());
        if (node.getMathMatrixAccessOperatorSymbol() != null)
            res.setMathMatrixAccessOperatorSymbol((MathMatrixAccessOperatorSymbol) get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getExpressionSymbol() != null) res.setExpressionSymbol(get(node.getExpressionSymbol()));
        res.setAssignmentOperator(node.getAssignmentOperator());
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathBooleanExpressionSymbol node) {
        MathBooleanExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathCompareExpressionSymbol node) {
        MathCompareExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setCompareOperator(node.getCompareOperator());
        if (node.getLeftExpression() != null) res.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) res.setRightExpression(get(node.getRightExpression()));
        res.setOperator(node.getOperator());
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathConditionalExpressionsSymbol node) {
        MathConditionalExpressionsSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getIfConditionalExpression() != null)
            res.setIfConditionalExpression((MathConditionalExpressionSymbol) get(node.getIfConditionalExpression()));
        res.setIfElseConditionalExpressions(new LinkedList<>());
        for (int i = 0; i < node.getIfElseConditionalExpressions().size(); i++) {
            res.getIfElseConditionalExpressions().add((MathConditionalExpressionSymbol)
                    get(node.getIfElseConditionalExpressions().get(i)));
        }
        if (node.getElseConditionalExpression().isPresent())
            res.setElseConditionalExpression((MathConditionalExpressionSymbol) get(node.getElseConditionalExpression().get()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathConditionalExpressionSymbol node) {
        MathConditionalExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getCondition().isPresent()) res.setCondition(get(node.getCondition().get()));
        res.setBodyExpressions(new LinkedList<>());
        for (int i = 0; i < node.getBodyExpressions().size(); i++) {
            res.getBodyExpressions().add(get(node.getBodyExpressions().get(i)));
        }
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathForLoopHeadSymbol node) {
        MathForLoopHeadSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setNameLoopVariable(node.getNameLoopVariable());
        if (node.getMathExpression() != null) res.setMathExpression(get(node.getMathExpression()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathForLoopExpressionSymbol node) {
        MathForLoopExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getForLoopHead() != null) res.setForLoopHead((MathForLoopHeadSymbol) get(node.getForLoopHead()));
        for (int i = 0; i < node.getForLoopBody().size(); i++) {
            res.addForLoopBody(get(node.getForLoopBody().get(i)));
        }
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathNameExpressionSymbol node) {
        MathNameExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setNameToResolveValue(node.getNameToResolveValue());
        res.setNameToAccess(node.getNameToAccess());
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathNumberExpressionSymbol node) {
        MathNumberExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setValue(node.getValue());
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathParenthesisExpressionSymbol node) {
        MathParenthesisExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getMathExpressionSymbol() != null) res.setMathExpressionSymbol(get(node.getMathExpressionSymbol()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathPreOperatorExpressionSymbol node) {
        MathPreOperatorExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getMathExpressionSymbol() != null) res.setMathExpressionSymbol(get(node.getMathExpressionSymbol()));
        res.setOperator(node.getOperator());
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathValueSymbol node) {
        MathValueSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setMatrixProperties(node.getMatrixProperties());
        if (node.getType() != null) res.setType((MathValueType) get(node.getType()));
        if (node.getValue() != null) res.setValue(get(node.getValue()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathValueType node) {
        MathValueType res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setProperties(node.getProperties());
        res.setType(node.getType());
        res.setTypeRef(node.getTypeRef());
        res.setDimensions(new LinkedList<>());
        for (int i = 0; i < node.getDimensions().size(); i++) {
            res.getDimensions().add(get(node.getDimensions().get(i)));
        }
        copyMap.put(node, res);
    }

    @Override public void endVisit(MathMatrixAccessOperatorSymbol node) {
        MathMatrixAccessOperatorSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setAccessStartSymbol(node.getAccessStartSymbol());
        res.setAccessEndSymbol(node.getAccessEndSymbol());
        if (node.getMathMatrixNameExpressionSymbol() != null)
            res.setMathMatrixNameExpressionSymbol(get(node.getMathMatrixNameExpressionSymbol()));
        res.setMathMatrixAccessSymbols(new LinkedList<>());
        for (int i = 0; i < node.getMathMatrixAccessSymbols().size(); i++) {
            res.getMathMatrixAccessSymbols().add(get(node.getMathMatrixAccessSymbols().get(i)));
        }
        copyMap.put(node, res);
    }

    @Override public void endVisit(MathMatrixNameExpressionSymbol node) {
        MathMatrixNameExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.isASTMathMatrixNamePresent())
            res.setAstMathMatrixNameExpression(node.getAstMathMatrixNameExpression());
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            res.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        res.setNameToAccess(node.getNameToAccess());
        copyMap.put(node, res);
    }

    @Override public void endVisit(MathMatrixVectorExpressionSymbol node) {
        MathMatrixVectorExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getStart() != null) res.setStart(get(node.getStart()));
        if (node.getStep().isPresent())
            res.setStep(get(node.getStep().get()));
        if (node.getEnd() != null) res.setEnd(get(node.getEnd()));
        copyMap.put(node, res);
    }

    @Override public void endVisit(MathMatrixArithmeticValueSymbol node) {
        MathMatrixArithmeticValueSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setMatrixProperties(node.getMatrixProperties());
        res.setVectors(new LinkedList<>());
        for (int i = 0; i < node.getVectors().size(); i++) {
            res.getVectors().add((MathMatrixAccessOperatorSymbol) get(node.getVectors().get(i)));
        }
        copyMap.put(node, res);
    }

    @Override public void endVisit(MathMatrixAccessSymbol node) {
        MathMatrixAccessSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getMathExpressionSymbol().isPresent())
            res.setMathExpressionSymbol(get(node.getMathExpressionSymbol().get()));
        copyMap.put(node, res);
    }

    @Override public void endVisit(MathMatrixArithmeticExpressionSymbol node) {
        MathMatrixArithmeticExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        res.setOperator(node.getOperator());
        res.setMathOperator(node.getMathOperator());
        if (node.getLeftExpression() != null) res.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) res.setRightExpression(get(node.getRightExpression()));
        copyMap.put(node, res);
    }

    protected Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

    @Override
    public Set<MathExpressionSymbol> getVisitedSymbols() {
        return visitedSymbols;
    }
}
