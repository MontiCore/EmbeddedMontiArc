/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._matrixprops.MatrixProperties;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class MathMatrixArithmeticValueSymbol extends MathMatrixExpressionSymbol {

    protected List<MathMatrixAccessOperatorSymbol> vectors = new ArrayList<>();

    protected ArrayList<MatrixProperties> matrixProperties = new ArrayList<>();

    public MathMatrixArithmeticValueSymbol() {

    }

    public List<MathMatrixAccessOperatorSymbol> getVectors() {
        return vectors;
    }

    public void setVectors(List<MathMatrixAccessOperatorSymbol> vectors) {
        this.vectors = vectors;
    }

    public void addMathMatrixAccessSymbol(MathMatrixAccessOperatorSymbol vector) {
        vectors.add(vector);
    }

    public void setMatrixProperties(ArrayList<MatrixProperties> matrixProperties) { this.matrixProperties = matrixProperties; }

    public ArrayList<MatrixProperties> getMatrixProperties() { return matrixProperties; }

    @Override
    public String getTextualRepresentation() {
        String result = "[";
        int counter = 0;
        for (MathMatrixAccessOperatorSymbol symbol : vectors) {
            result += symbol.getTextualRepresentation();
            ++counter;
            if (vectors.size() > counter)
                result += ";";
        }
        result += "]";
        return result;
    }

    @Override
    public boolean isValueExpression() {
        return true;
    }


}
