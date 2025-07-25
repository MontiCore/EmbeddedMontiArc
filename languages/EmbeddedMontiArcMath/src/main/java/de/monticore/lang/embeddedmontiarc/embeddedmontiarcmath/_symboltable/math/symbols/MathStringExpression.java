/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class MathStringExpression extends MathExpressionSymbol {
    public static int ID = 10001;

    protected String text;
    protected List<MathMatrixAccessSymbol> previousExpressionSymbols = new ArrayList<>();

    public MathStringExpression() {
        setID(ID);
    }

    public MathStringExpression(String text, List<MathMatrixAccessSymbol> previousExpressionSymbol) {
        this.text = text;
        setID(ID);
        if (previousExpressionSymbol != null)
            this.previousExpressionSymbols.addAll(previousExpressionSymbol);
    }

    public String getText() {
        return text;
    }

    public void setText(String text) {
        this.text = text;
    }

    @Override
    public String getTextualRepresentation() {
        return text;
    }

    public List<MathMatrixAccessSymbol> getPreviousExpressionSymbols() {
        return previousExpressionSymbols;
    }

    public void setPreviousExpressionSymbols(List<MathMatrixAccessSymbol> previousExpressionSymbols) {
        this.previousExpressionSymbols = previousExpressionSymbols;
    }
}
