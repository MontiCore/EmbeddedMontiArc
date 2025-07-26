/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 */
public class MathChainedExpression extends MathExpressionSymbol {
    public static int ID = 10002;
    protected MathExpressionSymbol firstExpressionSymbol, secondExpressionSymbol;

    public MathChainedExpression() {
        setID(ID);
    }

    public MathChainedExpression(MathExpressionSymbol firstExpressionSymbol, MathExpressionSymbol secondExpressionSymbol) {
        this.firstExpressionSymbol = firstExpressionSymbol;
        this.secondExpressionSymbol = secondExpressionSymbol;
        setID(ID);
    }


    public MathExpressionSymbol getFirstExpressionSymbol() {
        return firstExpressionSymbol;
    }

    public void setFirstExpressionSymbol(MathExpressionSymbol firstExpressionSymbol) {
        this.firstExpressionSymbol = firstExpressionSymbol;
    }

    public MathExpressionSymbol getSecondExpressionSymbol() {
        return secondExpressionSymbol;
    }

    public void setSecondExpressionSymbol(MathExpressionSymbol secondExpressionSymbol) {
        this.secondExpressionSymbol = secondExpressionSymbol;
    }

    @Override
    public String getTextualRepresentation() {
        return firstExpressionSymbol.getTextualRepresentation() + secondExpressionSymbol.getTextualRepresentation();
    }

}
