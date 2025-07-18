/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 */
public class MathMatrixAccessOperatorSymbol extends MathMatrixExpressionSymbol {
    protected MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol;
    protected List<MathMatrixAccessSymbol> mathMatrixAccessSymbols = new ArrayList<>();
    protected String accessStartSymbol = "(";
    protected String accessEndSymbol = ")";

    public MathMatrixAccessOperatorSymbol() {
        super();
    }

    public MathMatrixNameExpressionSymbol getMathMatrixNameExpressionSymbol() {
        return mathMatrixNameExpressionSymbol;
    }

    public void setMathMatrixNameExpressionSymbol(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol) {
        this.mathMatrixNameExpressionSymbol = mathMatrixNameExpressionSymbol;
    }

    public String getAccessStartSymbol() {
        return accessStartSymbol;
    }

    public void setAccessStartSymbol(String accessStartSymbol) {
        this.accessStartSymbol = accessStartSymbol;
    }

    public String getAccessEndSymbol() {
        return accessEndSymbol;
    }

    public void setAccessEndSymbol(String accessEndSymbol) {
        this.accessEndSymbol = accessEndSymbol;
    }

    public boolean isDoubleDot(int index) {
        return mathMatrixAccessSymbols.get(index).isDoubleDot();
    }

    public Optional<MathExpressionSymbol> getMathMatrixAccessSymbol(int index) {
        return mathMatrixAccessSymbols.get(index).getMathExpressionSymbol();
    }

    public List<MathMatrixAccessSymbol> getMathMatrixAccessSymbols() {
        return mathMatrixAccessSymbols;
    }

    public void setMathMatrixAccessSymbols(List<MathMatrixAccessSymbol> mathMatrixAccessSymbols) {
        this.mathMatrixAccessSymbols = mathMatrixAccessSymbols;
    }

    public void addMathMatrixAccessSymbol(MathMatrixAccessSymbol symbol) {
        mathMatrixAccessSymbols.add(symbol);
    }

    @Override
    public String getTextualRepresentation() {
        String result = accessStartSymbol;
        int counter = 0;
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixAccessSymbols) {
            result += accessSymbol.getTextualRepresentation();
            ++counter;
            if (counter < mathMatrixAccessSymbols.size())
                result += ", ";
        }
        result += accessEndSymbol;

        return result;
    }


}
