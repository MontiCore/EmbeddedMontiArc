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


import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.linear.*;


import java.util.ArrayList;

/**
 * Created by Philipp Goerick on 07.07.2017.
 *
 * Identifies matrix properties
 */
public class MatrixPropertiesIdentifier {

    private Array2DRowFieldMatrix<Complex> matrix;

    private ArrayList<MatrixProperties> props;

    public MatrixPropertiesIdentifier(MathMatrixArithmeticValueSymbol symbol){
        this.matrix = ConstructComplexMatrix.constructComplexMatrix(symbol);
        props = new ArrayList<>();
    }

    /**
     * Identify the matrix properties
     * @return matrix element that astMatrix has properties of
     */
    public ArrayList<MatrixProperties> identifyMatrixProperties(){
        if (checkPositive(matrix)) props.add(MatrixProperties.Positive);
        else if (checkNegative(matrix)) props.add(MatrixProperties.Negative);
        if(matrix.isSquare()) squareMatrix();
        return props;
    }

    private void squareMatrix() {
        props.add(MatrixProperties.Square);
        if (identifyNonSingularMatrix()) props.add(MatrixProperties.Invertible);
        if (identifyNormalMatrix()) NormMatrix();
    }

    private void NormMatrix() {
        props.add(MatrixProperties.Norm);
        if (identifyDiagMatrix()) props.add(MatrixProperties.Diag);
        if (identifyHermitianMatrix()) hermMatrix();
        if (identifySkewHermitianMatrix()) props.add(MatrixProperties.SkewHerm);
    }

    private void hermMatrix() {
        props.add(MatrixProperties.Herm);
        props = IdentifyDefiniteHelper.identifyDefMatrix(matrix, props);
    }

    /**
     * check if matrix is diagonal
     * @return true if diagonal, else false
     */
    private boolean identifyDiagMatrix(){
        for (int i = 0; i < matrix.getRowDimension(); i++){
            if (checkColumnDiag(i)) return false;
        }
        return true;
    }

    private boolean checkColumnDiag(int i) {
        for (int j = 0; j < matrix.getColumnDimension(); j++) {
            if(i != j){
                if (checkEntryDiag(i, j)) return true;
            }
        }
        return false;
    }

    private boolean checkEntryDiag(int i, int j) {
        if(!(matrix.getEntry(i,j).equals(Complex.ZERO))) return true;
        return false;
    }

    /**
     * checks if matrix is normal
     * @return true if normal, else false
     */
    private boolean identifyNormalMatrix() {
        FieldMatrix<Complex> conjugate_transpose = matrix.createMatrix(matrix.getRowDimension(),matrix.getColumnDimension());
        for (int i = 0; i < matrix.getRowDimension(); i++) {
            for (int j = 0; j < matrix.getColumnDimension(); j++) {
                Complex value = matrix.getEntry(i,j).conjugate();
                conjugate_transpose.setEntry(j,i,value);
            }
        }
        return matrix.multiply(conjugate_transpose).equals(conjugate_transpose.multiply(matrix));
    }

    /**
     * check if matrix is hermitian
     * @return true if hermitian, else false
     */
    private boolean identifyHermitianMatrix(){
        for (int i = 0; i < matrix.getRowDimension(); i++) {
            if (checkColumnHerm(i, true)) return false;
        }
        return true;
    }

    private boolean checkColumnHerm(int i, boolean herm) {
        for (int j = i; j < matrix.getColumnDimension(); j++){
            if (herm){
                if (checkEntryHerm(i, j)) return true;
            }
            else if (checkEntrySkewHerm(i, j)) return true;
        }
        return false;
    }

    private boolean checkEntryHerm(int i, int j) {
        if (i == j) {
            if(matrix.getEntry(i,j).getImaginary() != 0) return true;
        }
        else {
            if (matrix.getEntry(i,j).getImaginary() != 0 - matrix.getEntry(j,i).getImaginary()
                    || matrix.getEntry(i,j).getReal() != matrix.getEntry(j,i).getReal()) return true;
        }
        return false;
    }

    private boolean identifySkewHermitianMatrix(){
        for (int i = 0; i < matrix.getRowDimension(); i++) {
            if (checkColumnHerm(i, false)) return false;
        }
        return true;
    }

    private boolean checkEntrySkewHerm(int i, int j) {
        if (i == j) {
            if(matrix.getEntry(i,j).getReal() != 0) return true;
        }
        else {
            if (matrix.getEntry(i,j).getReal() != 0 - matrix.getEntry(j,i).getReal()
                    || matrix.getEntry(i,j).getImaginary() != matrix.getEntry(j,i).getImaginary()) return true;
        }
        return false;
    }

    /**
     * Check if matrix is non-singular(invertible) by checking if the determinant is not equal to zero
     * @return true if invertible, else false
     */
    private boolean identifyNonSingularMatrix(){
        return !(DeterminantCalculator.getDeterminant(matrix).equals(Complex.ZERO));
    }

    /**
     * Check if matrix has only positive values
     * @param m the matrix to check
     * return true if all values are positive, false else
     */
    private boolean checkPositive(Array2DRowFieldMatrix<Complex> m){
        for (int i = 0; i < m.getRowDimension(); i++) {
            if (checkColumnPosNeg(m, i, true)) return false;
        }
        return true;
    }

    private boolean checkColumnPosNeg(Array2DRowFieldMatrix<Complex> m, int i, boolean pos) {
        for (int j = 0; j < m.getColumnDimension(); j++) {
            if (pos){
                if (m.getEntry(i,j).getReal() < 0) return true;
            }
            else if (m.getEntry(i,j).getReal() > 0) return true;
            if (!(m.getEntry(i,j).getImaginary() == 0)) return true;
        }
        return false;
    }

    /**
     * Check if matrix has only negative values
     * @param m the matrix to check
     * return true if all values are negative, false else
     */
    private boolean checkNegative(Array2DRowFieldMatrix<Complex> m){
        for (int i = 0; i < m.getRowDimension(); i++) {
            if (checkColumnPosNeg(m, i, false)) return false;
        }
        return true;
    }
}

