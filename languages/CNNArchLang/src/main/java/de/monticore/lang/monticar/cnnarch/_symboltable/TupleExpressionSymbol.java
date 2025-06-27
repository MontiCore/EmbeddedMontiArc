/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.cnnarch.helper.Utils;

import java.util.LinkedList;
import java.util.List;

public class TupleExpressionSymbol extends MathExpressionSymbol {

    List<MathExpressionSymbol> expressions = new LinkedList<>();

    public TupleExpressionSymbol() {
    }

    public TupleExpressionSymbol(List<MathExpressionSymbol> expressions) {
        this.expressions = expressions;
    }

    @Override
    public String getTextualRepresentation() {
        return Utils.createTupleTextualRepresentation(getExpressions(), MathExpressionSymbol::getTextualRepresentation);
    }

    public void add(MathExpressionSymbol symbol){
        expressions.add(symbol);
    }

    public List<MathExpressionSymbol> getExpressions() {
        return expressions;
    }

    public void setExpressions(List<MathExpressionSymbol> expressions) {
        this.expressions = expressions;
    }
}
