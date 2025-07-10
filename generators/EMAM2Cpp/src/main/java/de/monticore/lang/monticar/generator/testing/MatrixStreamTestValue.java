/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.testing;

/**
 */
public class MatrixStreamTestValue<T> implements StreamTestValue {
    protected T[][] matrixValues;

    public MatrixStreamTestValue(T[][] matrixValues) {
        this.matrixValues = matrixValues;
    }


    public T[][] getMatrixValues() {
        return matrixValues;
    }

    public void setMatrixValues(T[][] matrixValues) {
        this.matrixValues = matrixValues;
    }

    @Override
    public String getStringRepresentation() {
        //TODO
        StringBuilder result = new StringBuilder();
        result.append("[");
        for (int i = 0; i < matrixValues.length; ++i) {
            for (int j = 0; j < matrixValues[0].length; ++j) {
                result.append(matrixValues[i][j]);
                if (j + 1 < matrixValues[0].length)
                    result.append(", ");
            }
            if (i + 1 < matrixValues.length)
                result.append("; ");
        }
        result.append("]");
        return result.toString();
    }
}
