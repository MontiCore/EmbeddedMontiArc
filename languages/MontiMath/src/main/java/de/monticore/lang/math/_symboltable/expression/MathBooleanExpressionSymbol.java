/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

public class MathBooleanExpressionSymbol extends MathValueExpressionSymbol {
    private boolean value;

    public MathBooleanExpressionSymbol(boolean value) {
        super();
        this.value = value;
    }

    @Override
    public boolean isBooleanExpression() {
        return true;
    }

    @Override
    public String getTextualRepresentation() {
        return value ? "true" : "false";
    }
}
