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
package de.monticore.lang.math._matrixprops;


import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
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
        if (checkPositive(true)) props.add(MatrixProperties.Positive);
        else if (checkPositive(false)) props.add(MatrixProperties.Negative);
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
        if (identifyHermitianMatrix(true)) hermMatrix();
        if (identifyHermitianMatrix(false)) props.add(MatrixProperties.SkewHerm);
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
            if (diagCondition(i, j)) return true;
        }
        return false;
    }

    private boolean diagCondition(int i, int j) {
        if(i != j){
            if (checkEntryDiag(i, j)) return true;
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
        FieldMatrix<Complex> conjugate_transpose = matrix.createMatrix(matrix.getRowDimension(), matrix.getColumnDimension());
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
    private boolean identifyHermitianMatrix(boolean herm){
        for (int i = 0; i < matrix.getRowDimension(); i++) {
            if (checkColumnHerm(i, herm)) return false;
        }
        return true;
    }

    private boolean checkColumnHerm(int i, boolean herm) {
        for (int j = i; j < matrix.getColumnDimension(); j++){
            if (hermCondition(i, herm, j)) return true;
        }
        return false;
    }

    private boolean hermCondition(int i, boolean herm, int j) {
        if (checkEntryHerm(i, j, herm)) return true;
        return false;
    }

    private boolean checkEntryHerm(int i, int j, boolean herm) {
        double real1 = matrix.getEntry(i, j).getReal();
        double img1 = matrix.getEntry(i, j).getImaginary();
        double real2 = matrix.getEntry(j, i).getReal();
        double img2 = matrix.getEntry(j, i).getImaginary();
        boolean res = false;
        if (i == j) {
            if (herm) {
                if (img1 != 0) res = true;
            } else if (real1 != 0) res = true;
        } else {
            if (herm) {
                if (img1 != 0 - img2 || real1 != real2) res = true;
            } else {
                if (real1 != 0 - real2 || img1 != img2) res = true;
            }
        }
        return res;
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
     * @param pos if positive or negative should be checked
     * return true if all values are positive, false else
     */
    private boolean checkPositive(boolean pos){
        for (int i = 0; i < matrix.getRowDimension(); i++) {
            if (checkColumnPosNeg(i, pos)) return false;
        }
        return true;
    }

    private boolean checkColumnPosNeg(int i, boolean pos) {
        for (int j = 0; j < matrix.getColumnDimension(); j++) {
            if (posCondition(i, pos, j)) return true;
        }
        return false;
    }

    private boolean posCondition(int i, boolean pos, int j) {
        if (pos){
            if (matrix.getEntry(i,j).getReal() < 0) return true;
        }
        else if (matrix.getEntry(i,j).getReal() > 0) return true;
        if (!(matrix.getEntry(i,j).getImaginary() == 0)) return true;
        return false;
    }
}