/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialGuessSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialValueSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.copy.CopyMathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

public class EMAMCopyMathExpressionSymbol extends CopyMathExpressionSymbol implements EMAMMathExpressionSymbolVisitor {

    public static MathStatementsSymbol copy(MathStatementsSymbol symbol) {
        instance = new EMAMCopyMathExpressionSymbol();
        return CopyMathExpressionSymbol.copy(symbol);
    }

    public static <T extends MathExpressionSymbol> T copy(T symbol) {
        instance = new EMAMCopyMathExpressionSymbol();
        return CopyMathExpressionSymbol.copy(symbol);
    }

    @Override
    protected CopyMathExpressionSymbol instantiate() {
        return new EMAMCopyMathExpressionSymbol();
    }

    @Override
    protected <T extends MathExpressionSymbol> T get(T symbol) {
        MathExpressionSymbol copy = copyMap.get(symbol);
        if (copy != null) return (T) copy;
        if (symbol instanceof EMAMEquationSymbol)
            copy = new EMAMEquationSymbol();
        else if (symbol instanceof EMAMInitialGuessSymbol)
            copy = new EMAMInitialGuessSymbol(((EMAMInitialGuessSymbol) symbol).getNameToAccess());
        else if (symbol instanceof EMAMInitialValueSymbol)
            copy = new EMAMInitialValueSymbol(((EMAMInitialValueSymbol) symbol).getNameToAccess());
        else
            return super.get(symbol);

        leftToCopy.put(copy, symbol);
        copyMap.put(symbol, copy);
        return (T) copy;
    }

    @Override
    public void endVisit(EMAMEquationSymbol node) {
        EMAMEquationSymbol res = new EMAMEquationSymbol();
        copyMathExpressionSymbol(res, node);
        if (node.getLeftExpression() != null) res.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) res.setRightExpression(get(node.getRightExpression()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(EMAMInitialGuessSymbol node) {
        EMAMInitialGuessSymbol res = new EMAMInitialGuessSymbol(node.getNameToAccess());
        copyMathExpressionSymbol(res, node);
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            res.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getValue() != null)
            res.setValue(get(node.getValue()));
        copyMap.put(node, res);
    }

    @Override
    public void endVisit(EMAMInitialValueSymbol node) {
        EMAMInitialValueSymbol res = new EMAMInitialValueSymbol(node.getNameToAccess());
        copyMathExpressionSymbol(res, node);
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            res.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getValue() != null)
            res.setValue(get(node.getValue()));
        copyMap.put(node, res);
    }
}
