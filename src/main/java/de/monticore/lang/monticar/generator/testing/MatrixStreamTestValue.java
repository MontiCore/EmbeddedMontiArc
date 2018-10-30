/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator.testing;

/**
 * @author Sascha Schneiders
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
