/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.symboltable.Scope;

import javax.measure.unit.Unit;
import java.util.Optional;

/**
 */
public class MathValueReference extends MathVariableDeclarationSymbolReference implements MathValue {
    /** every math value has a unit */
    private Unit unit = Unit.ONE;

    /** flag for incompatible units */
    private boolean incompUnit=false;

    /** flag for end operator */
    private boolean isEndoperator = false;

    /** Matrix index reference */
    Optional<MathValue> MatrixIndex = Optional.empty();

    public MathValueReference(String name, Scope enclosingScopeOfReference) {
        super(name, enclosingScopeOfReference);
    }

    /**
     * @return unit for this math value
     */
    @Override
    public Unit getUnit() {
        return unit;
    }

    /**
     * set a new unit for this math value
     *
     * @param unit (jscience unit)
     */
    @Override
    public void setUnit(Unit unit) {
            this.unit=unit;
    }

    /**
     * check if the units are incompatible
     *
     * @return TRUE, if the units are incompatible, otherwise FALSE
     */
    @Override
    public boolean isIncompUnit() {
        return incompUnit;
    }

    /**
     * set the flag to check if the units are incompatible
     *
     * @param x set TRUE if the math value/expression has incompatible units, otherwise FALSE
     */
    @Override
    public void setIncompUnit(boolean x) {
        incompUnit = x;
    }

    /**
     * checks the end operator
     *
     * @return TRUE, if there exists one, otherwise FALSE
     */
    public boolean isEndoperator() {
        return isEndoperator;
    }

    /**
     * set TRUE if there exists a end operator, otherwise FALSE
     */
    public void setEndoperator(boolean x) {
        isEndoperator = x;
    }
    /**
     * get the matrix index
     */
    public Optional<MathValue> getMatrixIndex() {
        return MatrixIndex;
    }

    /**
     * set the index for this matrix
     */
    public void setMatrixIndex(Optional<MathValue> matrixIndex) {
        MatrixIndex = matrixIndex;
    }

    /**
     * converts the math value reference to a String {@link String}
     *
     * @return the String representation of the math value reference
     *
     * @detail: first the name is printed, if its matrix reference
     * we have more cases
     * 1. something like A(1) or 2 dim. A(1,3)
     * 2. something with the endVal operator like A(1:3, 2:4), A(:, 1:2) or A(2:6, :)
     *
     * we used the notation from matlab
     */
    @Override
    public String toString(){
        String output = "";

        output += this.getName();

        if(MatrixIndex.isPresent()){
                output += "(" + getMatrixIndex() + ")";
        }

        return output;
    }
}
//
