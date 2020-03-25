/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.Optional;

/**
 */
public class MathMatrixAccessSymbol extends MathMatrixExpressionSymbol {

    protected Optional<MathExpressionSymbol> mathExpressionSymbol = Optional.empty();

    public MathMatrixAccessSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = Optional.of(mathExpressionSymbol);
    }

    //: Access per default
    public MathMatrixAccessSymbol() {
        super();
    }


    @Override
    public String getTextualRepresentation() {
        String result = "";
        if (mathExpressionSymbol.isPresent()) {
            result += mathExpressionSymbol.get().getTextualRepresentation();
        } else {
            result += ":";
        }
        return result;
    }

    public Optional<MathExpressionSymbol> getMathExpressionSymbol() {
        return mathExpressionSymbol;
    }

    public void setMathExpressionSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = Optional.ofNullable(mathExpressionSymbol);
    }

    public boolean isDoubleDot() {
        return !mathExpressionSymbol.isPresent();
    }

    @Override
    public boolean isMatrixAccessExpression() {
        return true;
    }
}
