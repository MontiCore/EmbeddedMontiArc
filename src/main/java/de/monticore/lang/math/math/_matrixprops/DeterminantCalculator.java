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
package de.monticore.lang.math.math._matrixprops;

import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.complex.ComplexField;
import org.apache.commons.math3.linear.Array2DRowFieldMatrix;

public class DeterminantCalculator {

    public static Complex getDeterminant(Array2DRowFieldMatrix<Complex> m){
        if (m.getRowDimension() == 1) return m.getEntry(0,0);
        Complex value = getValue(m);
        return value;
    }

    private static Complex getValue(Array2DRowFieldMatrix<Complex> m) {
        Complex value = new Complex(0);
        for (int i = 0; i < m.getRowDimension(); i++) {
            Complex sign = new Complex(Math.pow(-1,i));
            Array2DRowFieldMatrix<Complex> m_sub = new Array2DRowFieldMatrix<>(ComplexField.getInstance(), m.getRowDimension() - 1, m.getColumnDimension() - 1);
            goThroughRow(m, i, m_sub);
            value = value.add(sign.multiply(m.getEntry(i,0).multiply(getDeterminant(m_sub))));
        }
        return value;
    }

    private static void goThroughRow(Array2DRowFieldMatrix<Complex> m, int i, Array2DRowFieldMatrix<Complex> m_sub) {
        for (int j = 0; j < m.getRowDimension(); j++) {
            if (j < i){
                lowerEntry(m, m_sub, j);
            }
            else if (j > i){
                upperEntry(m, m_sub, j);
            }
        }
    }

    private static void upperEntry(Array2DRowFieldMatrix<Complex> m, Array2DRowFieldMatrix<Complex> m_sub, int j) {
        for (int k = 0; k < m.getColumnDimension()-1; k++) {
            m_sub.setEntry(j-1,k,m.getEntry(j,k+1));
        }
    }

    private static void lowerEntry(Array2DRowFieldMatrix<Complex> m, Array2DRowFieldMatrix<Complex> m_sub, int j) {
        for (int k = 0; k < m.getColumnDimension()-1; k++) {
            m_sub.setEntry(j,k,m.getEntry(j,k+1));
        }
    }
}
