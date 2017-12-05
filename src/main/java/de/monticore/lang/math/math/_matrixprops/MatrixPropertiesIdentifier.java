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

import de.monticore.lang.math.math._symboltable.JSValue;
import de.monticore.lang.math.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import org.apache.commons.math4.complex.Complex;
import org.apache.commons.math4.complex.ComplexField;
import org.apache.commons.math4.linear.*;
import org.apache.commons.math4.linear.Array2DRowRealMatrix;
import org.apache.commons.math4.linear.EigenDecomposition;
import org.apache.commons.math4.linear.RealMatrix;

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
        Complex[][] c = new Complex[symbol.getVectors().size()][symbol.getVectors().get(0).getMathMatrixAccessSymbols().size()];
        for (int i = 0; i < c.length; i++) {
            for (int j = 0; j < c[0].length; j++) {
                MathMatrixAccessOperatorSymbol innerVector = symbol.getVectors().get(i);
                if(innerVector.getMathMatrixAccessSymbol(j).isPresent()) {
                    if (innerVector.getMathMatrixAccessSymbol(j).get().isValueExpression()) {
                        JSValue value = ((MathNumberExpressionSymbol) innerVector.getMathMatrixAccessSymbol(j).get().getAssignedMathExpressionSymbol()).getValue();
                        if (value.getImagNumber().isPresent()) {
                            c[i][j] = new Complex(value.getRealNumber().doubleValue(), value.getImagNumber().get().doubleValue());
                        } else {
                            c[i][j] = new Complex((value.getRealNumber().doubleValue()));
                        }
                    }
                    /*else if(innerVector.getMathMatrixAccessSymbol(j).get().isArithmeticExpression()) {
                        MathArithmeticExpressionSymbol exp = ((MathArithmeticExpressionSymbol)innerVector.getMathMatrixAccessSymbol(j).get());
                        if (exp.getMathOperator().equals("+") || exp.getMathOperator().equals("-")) {
                            Complex valueRight = castToComplex(exp.getRightExpression());
                            c[i][j] = dissolveDashMathExpression(exp, valueRight);
                        }else{
                            c[i][j] = dissolveDotMathExpression(exp);
                        }
                    }*/
                    else c[i][j] = new Complex(0);
                }else c[i][j] = new Complex(0);
            }
        }
        this.matrix = new Array2DRowFieldMatrix<>(ComplexField.getInstance(), c);
        props = new ArrayList<>();
    }

    private Complex dissolveDotMathExpression(MathArithmeticExpressionSymbol exp) {
        Complex valueRight = castToComplex(exp.getRightExpression());
        if (exp.getLeftExpression().isArithmeticExpression()) {
            if (((MathArithmeticExpressionSymbol) exp.getLeftExpression()).getMathOperator().equals("+") ||
                    ((MathArithmeticExpressionSymbol) exp.getLeftExpression()).getMathOperator().equals("-")) {
                Complex val = castToComplex(((MathArithmeticExpressionSymbol) exp.getLeftExpression()).getRightExpression());

                switch (exp.getMathOperator()) {
                    case "*": {
                        val = val.multiply(valueRight);
                    }
                    case "/": {
                        val = val.divide(valueRight);
                    }
                    case "^": {
                        val = val.pow(valueRight);
                    }
                }
                return dissolveDashMathExpression(((MathArithmeticExpressionSymbol)exp.getLeftExpression()), val);
            }

            switch (exp.getMathOperator()) {
                case "*": {
                    return dissolveDotMathExpression(((MathArithmeticExpressionSymbol) exp.getLeftExpression()))
                            .multiply(valueRight);
                }
                case "/": {
                    return dissolveDotMathExpression(((MathArithmeticExpressionSymbol) exp.getLeftExpression()))
                            .divide(valueRight);
                }
                case "^": {
                    return dissolveDotMathExpression(((MathArithmeticExpressionSymbol) exp.getLeftExpression()))
                            .pow(valueRight);
                }
                default: {
                    return valueRight;
                }
            }
        }

        Complex valueLeft = castToComplex(exp.getLeftExpression());
        switch (exp.getMathOperator()) {
            case "*": {
                return valueLeft.multiply(valueRight);
            }
            case "/": {
                return valueLeft.divide(valueRight);
            }
            case "^": {
                return valueLeft.pow(valueRight);
            }
            default: {
                return valueLeft;
            }
        }
    }

    private Complex dissolveDashMathExpression(MathArithmeticExpressionSymbol exp, Complex valueRight){
        if (exp.getLeftExpression().isArithmeticExpression()) {
            if (((MathArithmeticExpressionSymbol) exp.getLeftExpression()).getMathOperator().equals("+") ||
                    ((MathArithmeticExpressionSymbol) exp.getLeftExpression()).getMathOperator().equals("-")) {
                Complex newRight = castToComplex(((MathArithmeticExpressionSymbol)
                        exp.getLeftExpression()).getRightExpression());
                switch (exp.getMathOperator()){
                    case "+": {
                    return dissolveDashMathExpression(((MathArithmeticExpressionSymbol)exp.getLeftExpression()),
                            newRight.add(valueRight));
                    }
                    case "-": {
                        return dissolveDashMathExpression(((MathArithmeticExpressionSymbol)exp.getLeftExpression()),
                                newRight.subtract(valueRight));
                    }
                }
            }
            else{
                switch (exp.getMathOperator()) {
                    case "+": {
                        return dissolveDotMathExpression(((MathArithmeticExpressionSymbol)exp.getLeftExpression()))
                                .add(valueRight);
                    }
                    case "-": {
                        return (dissolveDotMathExpression(((MathArithmeticExpressionSymbol)exp.getLeftExpression())))
                                .subtract(valueRight);
                    }
                }
            }
        }
        switch (exp.getMathOperator()) {
            case "+": {
                return castToComplex(exp.getLeftExpression()).add(valueRight);
            }
            case "-": {
                return castToComplex(exp.getLeftExpression()).subtract(valueRight);
            }
            default:{
                return valueRight;
            }
        }
    }

    private Complex castToComplex(MathExpressionSymbol sym){
        JSValue Value = ((MathNumberExpressionSymbol)sym).getValue();
        if (Value.getImagNumber().isPresent()) {
            return new Complex(Value.getRealNumber().doubleValue(), Value.getImagNumber().get().doubleValue());
        } else {
            return new Complex(Value.getRealNumber().doubleValue());
        }
    }

    /**
     * Identify the matrix properties
     * @return matrix element that astMatrix has properties of
     */
    public ArrayList<MatrixProperties> identifyMatrixProperties(){
        if(matrix.isSquare()) {
            props.add(MatrixProperties.Square);
            if (identifyNonSingularMatrix()) props.add(MatrixProperties.Invertible);

            if (identifyNormalMatrix()) {
                props.add(MatrixProperties.Norm);
                if (identifyDiagMatrix()) props.add(MatrixProperties.Diag);

                if (identifyHermitianMatrix()) {
                    props.add(MatrixProperties.Herm);
                    identifyDefMatrix();
                }

                if (identifySkewHermitianMatrix()) props.add(MatrixProperties.SkewHerm);
            }
        }
        return props;
    }

    /**
     * check if matrix is diagonal
     * @return true if diagonal, else false
     */
    private boolean identifyDiagMatrix(){
        for (int i = 0; i < matrix.getRowDimension(); i++){
            for (int j = 0; j < matrix.getColumnDimension(); j++) {
                if(i != j){
                    if(!(matrix.getEntry(i,j).equals(Complex.ZERO))) return false;
                }
            }
        }
        return true;
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
            for (int j = i; j < matrix.getColumnDimension(); j++){
                if (i == j) {
                    if(matrix.getEntry(i,j).getImaginary() != 0) return false;
                }
                else {
                    if (matrix.getEntry(i,j).getImaginary() != 0 - matrix.getEntry(j,i).getImaginary()
                            || matrix.getEntry(i,j).getReal() != matrix.getEntry(j,i).getReal()) return false;
                }
            }
        }
        return true;
    }

    private boolean identifySkewHermitianMatrix(){
        for (int i = 0; i < matrix.getRowDimension(); i++) {
            for (int j = i; j < matrix.getColumnDimension(); j++){
                if (i == j) {
                    if(matrix.getEntry(i,j).getReal() != 0) return false;
                }
                else {
                    if (matrix.getEntry(i,j).getReal() != 0 - matrix.getEntry(j,i).getReal()
                            || matrix.getEntry(i,j).getImaginary() != matrix.getEntry(j,i).getImaginary()) return false;
                }
            }
        }
        return true;
    }


    /**
     * check which definite matrix has
     * @return JSMatrix with identified definite properties of astMatrix
     */

    private void identifyDefMatrix(){
        double[][] d= new double[matrix.getRowDimension()][matrix.getColumnDimension()];
        for (int i = 0; i < d.length; i++) {
            for (int j = 0; j < d[0].length; j++) {
                d[i][j] = matrix.getEntry(i,j).getReal();
            }
        }

        RealMatrix matrix = new Array2DRowRealMatrix(d);
        EigenDecomposition e = new EigenDecomposition(matrix);

        if (e.hasComplexEigenvalues()){
            props.add(MatrixProperties.Indef);
            return;
        }

        boolean flag = true;
        for (double v: e.getRealEigenvalues()) {
            if(v <= 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.PosDef);
        }

        flag = true;
        for (double v: e.getRealEigenvalues()) {
            if(v < 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.PosSemDef);
            return;
        }

        flag = true;
        for (double v: e.getRealEigenvalues()) {
            if(v >= 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.NegDef);
        }

        flag = true;
        for (double v: e.getRealEigenvalues()) {
            if(v > 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.NegSemDef);
            return;
        }

        props.add(MatrixProperties.Indef);

    }


    /**
     * Check if matrix is non-singular(invertible) by checking if the determinant is not equal to zero
     * @return true if invertible, else false
     */
    private boolean identifyNonSingularMatrix(){
        return !(getDeterminant(matrix).equals(Complex.ZERO));
    }

    /**
     * Compute determinant of a matrix
     * @param m Matrix of which determinant is computed
     * @return Complex determinant of matrix
     */
    private Complex getDeterminant(Array2DRowFieldMatrix<Complex> m){
        if (m.getRowDimension() == 1) return m.getEntry(0,0);

        Complex value = new Complex(0);
        for (int i = 0; i < m.getRowDimension(); i++) {
            Complex sign = new Complex(Math.pow(-1,i));
            Array2DRowFieldMatrix<Complex> m_sub = new Array2DRowFieldMatrix<>(ComplexField.getInstance(), m.getRowDimension() - 1, m.getColumnDimension() - 1);
            for (int j = 0; j < m.getRowDimension(); j++) {
                if (j < i){
                    for (int k = 0; k < m.getColumnDimension()-1; k++) {
                        m_sub.setEntry(j,k,m.getEntry(j,k+1));
                    }
                }
                else if (j > i){
                    for (int k = 0; k < m.getColumnDimension()-1; k++) {
                        m_sub.setEntry(j-1,k,m.getEntry(j,k+1));
                    }
                }
            }
            value = value.add(sign.multiply(m.getEntry(i,0).multiply(getDeterminant(m_sub))));
        }
        return value;
    }

}

