/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 */
public class MathConditionalExpressionSymbol extends MathExpressionSymbol {
    protected Optional<MathExpressionSymbol> condition = Optional.empty();
    protected List<MathExpressionSymbol> bodyExpressions = new ArrayList<>();

    public Optional<MathExpressionSymbol> getCondition() {
        return condition;
    }

    public void setCondition(MathExpressionSymbol condition) {
        this.condition = Optional.of(condition);
    }

    public List<MathExpressionSymbol> getBodyExpressions() {
        return bodyExpressions;
    }

    public void setBodyExpressions(List<MathExpressionSymbol> bodyExpressions) {
        this.bodyExpressions = bodyExpressions;
    }

    public void addBodyExpression(MathExpressionSymbol bodyExpression) {
        bodyExpressions.add(bodyExpression);
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";

        if (condition.isPresent())
            result += "(" + condition.get().getTextualRepresentation() + ")";
        if (bodyExpressions.size() > 0) {
            result += "{\n";
            for (MathExpressionSymbol symbol : bodyExpressions) {
                result += symbol.getTextualRepresentation() + ";\n";
            }
            result += "}\n";
        }
        return result;
    }
}
