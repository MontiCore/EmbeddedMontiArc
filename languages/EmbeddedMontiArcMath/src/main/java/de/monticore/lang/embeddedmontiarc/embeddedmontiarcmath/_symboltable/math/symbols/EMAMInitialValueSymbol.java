/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;

import java.util.Optional;

public class EMAMInitialValueSymbol extends MathExpressionSymbol {

    protected String nameToAccess;

    protected Optional<MathMatrixAccessOperatorSymbol> mathMatrixAccessOperatorSymbol = Optional.empty();

    protected MathExpressionSymbol value;

    public EMAMInitialValueSymbol(String nameToAccess) {
        super();
        this.nameToAccess = nameToAccess;
    }

    @Override
    public String getTextualRepresentation() {
        return getNameWithArray() + "(t=0)=" + value.getTextualRepresentation();
    }

    public String getNameWithArray() {
        return nameToAccess
                + (mathMatrixAccessOperatorSymbol.isPresent() ?
                mathMatrixAccessOperatorSymbol.get().getTextualRepresentation() : "");
    }

    public MathExpressionSymbol getValue() {
        return value;
    }

    public void setValue(MathExpressionSymbol value) {
        this.value = value;
    }

    public String getNameToAccess() {
        return nameToAccess;
    }

    public void setNameToAccess(String nameToAccess) {
        this.nameToAccess = nameToAccess;
    }

    public boolean isMathMatrixAccessOperatorSymbolPresent() {
        return mathMatrixAccessOperatorSymbol.isPresent();
    }

    public MathMatrixAccessOperatorSymbol getMathMatrixAccessOperatorSymbol() {
        return mathMatrixAccessOperatorSymbol.get();
    }

    public void setMathMatrixAccessOperatorSymbol(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol) {
        this.mathMatrixAccessOperatorSymbol = Optional.ofNullable(mathMatrixAccessOperatorSymbol);
    }

}
