/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;

public class MassMatrixRepresentation {

    private final MathMatrixArithmeticValueSymbol massMatrix;
    private final MathMatrixAccessOperatorSymbol x;
    private final MathMatrixAccessOperatorSymbol x_start;
    private final MathMatrixAccessOperatorSymbol function;

    public MassMatrixRepresentation(MathMatrixArithmeticValueSymbol massMatrix, MathMatrixAccessOperatorSymbol x, MathMatrixAccessOperatorSymbol x_start, MathMatrixAccessOperatorSymbol function) {

        this.massMatrix = massMatrix;
        this.x = x;
        this.x_start = x_start;
        this.function = function;
    }

    public MathMatrixArithmeticValueSymbol getMassMatrix() {
        return massMatrix;
    }

    public MathMatrixAccessOperatorSymbol getX() {
        return x;
    }

    public MathMatrixAccessOperatorSymbol getFunction() {
        return function;
    }

    public MathMatrixAccessOperatorSymbol getX_start() {
        return x_start;
    }
}
