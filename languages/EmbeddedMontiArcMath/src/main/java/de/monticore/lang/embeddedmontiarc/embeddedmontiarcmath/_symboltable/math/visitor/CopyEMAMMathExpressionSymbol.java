/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.visitor.CopyMathExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathStatementsSymbolCopy;
import de.monticore.lang.mathopt._symboltable.visitor.CopyMathOptExpressionSymbol;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class CopyEMAMMathExpressionSymbol extends CopyMathOptExpressionSymbol implements EMAMMathExpressionSymbolVisitor {

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
        CopyEMAMMathExpressionSymbol copy = new CopyEMAMMathExpressionSymbol();
        copy.handle(symbol);
        T res = copy.get(symbol);
        return res;
    }

    @Override
    protected <T extends MathExpressionSymbol> T get(T symbol) {
        MathExpressionSymbol copy = copyMap.get(symbol);
        if (copy != null) return (T) copy;
        if (symbol instanceof EMAMSpecificationSymbol)
            copy = new EMAMSpecificationSymbol(new ArrayList<>(), new ArrayList<>());
        else if (symbol instanceof EMAMSymbolicVariableSymbol)
            copy = new EMAMSymbolicVariableSymbol("");
        else if (symbol instanceof EMAMEquationSymbol)
            copy = new EMAMEquationSymbol();
        else if (symbol instanceof EMAMInitialGuessSymbol)
            copy = new EMAMInitialGuessSymbol("");
        else if (symbol instanceof EMAMInitialValueSymbol)
            copy = new EMAMInitialValueSymbol("");
        else if (symbol instanceof MathStringExpression)
            copy = new MathStringExpression();
        else if (symbol instanceof MathChainedExpression)
            copy = new MathChainedExpression();
        else
            return super.get(symbol);

        copyMap.put(symbol, copy);
        return (T) copy;
    }


    @Override
    public void endVisit(EMAMSpecificationSymbol node) {
        EMAMSpecificationSymbol res = get(node);
        for (EMAMSymbolicVariableSymbol variable : node.getVariables())
            res.addVariable(get(variable));
        for (EMAMInitialValueSymbol initialValue : node.getInitialValues())
            res.addInitialValue(get(initialValue));
        for (EMAMInitialGuessSymbol initialGuess : node.getInitialGuesses())
            res.addInitialGuess(get(initialGuess));
        for (EMAMEquationSymbol equation : node.getEquations())
            res.addEquation(get(equation));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(EMAMSymbolicVariableSymbol node) {
        EMAMSymbolicVariableSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getName() != null) res.setName(node.getName());
        if (node.getPort().isPresent()) res.setPort(node.getPort().get());
        if (node.getType() != null) res.setType(get(node.getType()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(EMAMEquationSymbol node) {
        EMAMEquationSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getLeftExpression() != null) res.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) res.setRightExpression(get(node.getRightExpression()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(EMAMInitialGuessSymbol node) {
        EMAMInitialGuessSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getNameToAccess() != null) res.setNameToAccess(node.getNameToAccess());
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            res.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getValue() != null)
            res.setValue(get(node.getValue()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(EMAMInitialValueSymbol node) {
        EMAMInitialValueSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getNameToAccess() != null) res.setNameToAccess(node.getNameToAccess());
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            res.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getValue() != null)
            res.setValue(get(node.getValue()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathStringExpression node) {
        MathStringExpression res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getText() != null) res.setText(node.getText());
        for (MathMatrixAccessSymbol previousExpressionSymbol : node.getPreviousExpressionSymbols())
            res.getPreviousExpressionSymbols().add(get(previousExpressionSymbol));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(MathChainedExpression node) {
        MathChainedExpression res = get(node);
        copyMathExpressionSymbol(res, node);
        if (node.getFirstExpressionSymbol() != null)
            res.setFirstExpressionSymbol(get(node.getFirstExpressionSymbol()));
        if (node.getSecondExpressionSymbol() != null)
            res.setSecondExpressionSymbol(get(node.getSecondExpressionSymbol()));
        copyMap.put(node, res);
    }
}
