/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

/**
 */
public class MathNameExpressionSymbol extends MathValueExpressionSymbol implements IMathNamedExpression {

    protected String nameToResolveValue;

    public MathNameExpressionSymbol() {
        super();
    }

    public MathNameExpressionSymbol(String nameToResolveValue) {
        super();
        this.nameToResolveValue = nameToResolveValue;
    }

    public void setNameToAccess(String nameToAccess) {
        setNameToResolveValue(nameToAccess);
    }

    public String getNameToAccess() {
        return getNameToResolveValue();
    }

    public String getNameToResolveValue() {
        return nameToResolveValue;
    }

    public void setNameToResolveValue(String nameToResolveValue) {
        this.nameToResolveValue = nameToResolveValue;
    }

    @Override
    public String getTextualRepresentation() {
        return getNameToResolveValue();
    }

    public boolean isDottedName() {
        return nameToResolveValue.contains(".");
    }

    @Override
    public boolean isNameExpression() {
        return true;
    }
}
