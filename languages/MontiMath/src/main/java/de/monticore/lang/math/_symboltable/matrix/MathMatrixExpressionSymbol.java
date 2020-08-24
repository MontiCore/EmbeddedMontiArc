/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 *
 */
public abstract class MathMatrixExpressionSymbol extends MathExpressionSymbol {
    public MathMatrixExpressionSymbol() {
        super();
    }

    public MathMatrixExpressionSymbol(String name) {
        super(name);
    }

    @Override
    public boolean isMatrixExpression() {
        return true;
    }

    public boolean isMatrixVectorExpression() {
        return false;
    }

    public boolean isMatrixAccessExpression() {
        return false;
    }

    public boolean isMatrixArithmeticExpression() {
        return false;
    }

    public boolean isMatrixNameExpression() {
        return false;
    }

    public boolean isMatrixPreOperatorSymbol() {
        return false;
    }
}
