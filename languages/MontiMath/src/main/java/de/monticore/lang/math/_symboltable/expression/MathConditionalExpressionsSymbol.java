/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 */
public class MathConditionalExpressionsSymbol extends MathExpressionSymbol {
    protected MathConditionalExpressionSymbol ifConditionalExpression;
    protected List<MathConditionalExpressionSymbol> ifElseConditionalExpressions = new ArrayList<>();
    protected Optional<MathConditionalExpressionSymbol> elseConditionalExpression = Optional.empty();

    public MathConditionalExpressionSymbol getIfConditionalExpression() {
        return ifConditionalExpression;
    }

    public void setIfConditionalExpression(MathConditionalExpressionSymbol ifConditionalExpression) {
        this.ifConditionalExpression = ifConditionalExpression;
    }

    public List<MathConditionalExpressionSymbol> getIfElseConditionalExpressions() {
        return ifElseConditionalExpressions;
    }

    public void setIfElseConditionalExpressions(List<MathConditionalExpressionSymbol> ifElseConditionalExpressions) {
        this.ifElseConditionalExpressions = ifElseConditionalExpressions;
    }

    public Optional<MathConditionalExpressionSymbol> getElseConditionalExpression() {
        return elseConditionalExpression;
    }

    public void setElseConditionalExpression(MathConditionalExpressionSymbol elseConditionalExpression) {
        this.elseConditionalExpression = Optional.of(elseConditionalExpression);
    }

    public void addElseIfConditionalExpression(MathConditionalExpressionSymbol elseIfConditionalExpressionSymbol) {
        ifElseConditionalExpressions.add(elseIfConditionalExpressionSymbol);
    }

    @Override
    public String getTextualRepresentation() {
        String result = "If:";

        result += ifConditionalExpression.getTextualRepresentation();
        if (ifElseConditionalExpressions.size() > 0)
            result += "Else If:";
        for (MathConditionalExpressionSymbol symbol : ifElseConditionalExpressions) {
            result += symbol.getTextualRepresentation();
        }
        if (elseConditionalExpression.isPresent())
            result += "Else:" + elseConditionalExpression.get().getTextualRepresentation();

        return result;
    }

    @Override
    public boolean isConditionalsExpression() {
        return true;
    }

}
