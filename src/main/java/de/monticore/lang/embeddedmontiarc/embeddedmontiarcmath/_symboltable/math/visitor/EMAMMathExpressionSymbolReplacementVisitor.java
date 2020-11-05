/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialGuessSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialValueSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.mathopt._symboltable.visitor.MathOptExpressionSymbolReplacementVisitor;

import java.util.Map;
import java.util.function.Function;

public class EMAMMathExpressionSymbolReplacementVisitor extends MathOptExpressionSymbolReplacementVisitor
        implements EMAMMathExpressionSymbolVisitor {

    public EMAMMathExpressionSymbolReplacementVisitor(Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        super(replacementMap);
    }

    public EMAMMathExpressionSymbolReplacementVisitor(Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        super(replacementFunction);
    }

    @Override
    public void visit(EMAMEquationSymbol node) {
        if (node.getLeftExpression() != null) node.setLeftExpression(get(node.getLeftExpression()));
        if (node.getRightExpression() != null) node.setRightExpression(get(node.getRightExpression()));
    }

    @Override
    public void visit(EMAMInitialGuessSymbol node) {
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            node.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getValue() != null) node.setValue(get(node.getValue()));
    }

    @Override
    public void visit(EMAMInitialValueSymbol node) {
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            node.setMathMatrixAccessOperatorSymbol(get(node.getMathMatrixAccessOperatorSymbol()));
        if (node.getValue() != null) node.setValue(get(node.getValue()));
    }
}
