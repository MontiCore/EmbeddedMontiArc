/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

/**
 */
public abstract class MathValueExpressionSymbol extends MathExpressionSymbol {

    public MathValueExpressionSymbol() {
        super();
    }

    public MathValueExpressionSymbol(String name) {
        super(name);
    }

    @Override
    public boolean isValueExpression() {
        return true;
    }

    public boolean isNameExpression() {
        return false;
    }

    public boolean isNumberExpression() {
        return false;
    }

    public boolean isBooleanExpression() {
        return false;
    }
}
