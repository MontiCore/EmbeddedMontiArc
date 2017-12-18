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
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Symbol;
import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.complex.ComplexField;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.RealMatrix;

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
                    MathExpressionSymbol expression = innerVector.getMathMatrixAccessSymbol(j).get();
                    c[i][j] = dissolveMathExpression(expression);
                }
                else c[i][j] = new Complex(0);
            }
        }
        this.matrix = new Array2DRowFieldMatrix<>(ComplexField.getInstance(), c);
        props = new ArrayList<>();
    }



    /**
     * Identify the matrix properties
     * @return matrix element that astMatrix has properties of
     */
    public ArrayList<MatrixProperties> identifyMatrixProperties(){
        if (checkPositive(matrix)) props.add(MatrixProperties.Positive);
        else if (checkNegative(matrix)) props.add(MatrixProperties.Negative);
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

    /**
     * Check if matrix has only positive values
     * @param m the matrix to check
     * return true if all values are positive, false else
     */
    private boolean checkPositive(Array2DRowFieldMatrix<Complex> m){
        for (int i = 0; i < m.getRowDimension(); i++) {
            for (int j = 0; j < m.getColumnDimension(); j++) {
                if (m.getEntry(i,j).getReal() < 0) return false;
                if (!(m.getEntry(i,j).getImaginary() == 0)) return false;
            }
        }
        return true;
    }

    /**
     * Check if matrix has only negative values
     * @param m the matrix to check
     * return true if all values are negative, false else
     */
    private boolean checkNegative(Array2DRowFieldMatrix<Complex> m){
        for (int i = 0; i < m.getRowDimension(); i++) {
            for (int j = 0; j < m.getColumnDimension(); j++) {
                if (m.getEntry(i,j).getReal() > 0) return false;
                if (!(m.getEntry(i,j).getImaginary() == 0)) return false;
            }
        }
        return true;
    }


    //Helper methods for obtaining complex values

    private Complex dissolveMathExpression(MathExpressionSymbol exp){
        if (exp.isParenthesisExpression())
            exp = ((MathParenthesisExpressionSymbol)exp).getMathExpressionSymbol();

        if (exp.isValueExpression()) {
            if (((MathValueExpressionSymbol) exp).isNameExpression())
                exp = resolveName((MathNameExpressionSymbol) exp);
            if (exp.isValueExpression())
                return castToComplex((MathNumberExpressionSymbol) exp);
            else return dissolveMathExpression(exp);
        }

        Complex value1 = dissolveChildExpression(((MathArithmeticExpressionSymbol)exp).getLeftExpression());
        Complex value2 = dissolveChildExpression(((MathArithmeticExpressionSymbol)exp).getRightExpression());

        switch (((MathArithmeticExpressionSymbol)exp).getMathOperator()) {
            case "+": {
                return value1.add(value2);
            }
            case "-": {
                return value1.subtract(value2);
            }
            case "*": {
                return value1.multiply(value2);
            }
            case "/": {
                return value1.divide(value2);
            }
            case "^": {
                return value1.pow(value2);
            }
            default: {
                return value1;
            }
        }
    }

    private Complex castToComplex(MathNumberExpressionSymbol sym){
        JSValue Value = sym.getValue();
        if (Value.getImagNumber().isPresent()) {
            return new Complex(Value.getRealNumber().doubleValue(), Value.getImagNumber().get().doubleValue());
        } else {
            return new Complex(Value.getRealNumber().doubleValue());
        }
    }

    private Complex dissolveChildExpression(MathExpressionSymbol expressionSymbol){
        if (expressionSymbol.isParenthesisExpression())
            expressionSymbol = ((MathParenthesisExpressionSymbol)expressionSymbol).getMathExpressionSymbol();

        if (expressionSymbol.isValueExpression()) {
            if (((MathValueExpressionSymbol) expressionSymbol).isNameExpression()) {
                expressionSymbol = resolveName((MathNameExpressionSymbol)expressionSymbol);
            }

            if (expressionSymbol.isParenthesisExpression())
                expressionSymbol = ((MathParenthesisExpressionSymbol)expressionSymbol).getMathExpressionSymbol();

            if (((MathValueExpressionSymbol)expressionSymbol).isNumberExpression())
                return castToComplex((MathNumberExpressionSymbol)expressionSymbol);
            else if (expressionSymbol.isArithmeticExpression())
                return dissolveMathExpression(expressionSymbol);

        }else if (expressionSymbol.isArithmeticExpression())
            return dissolveMathExpression(expressionSymbol);

        return new Complex(0);
    }

    private MathExpressionSymbol resolveName(MathNameExpressionSymbol expressionSymbol){
        Symbol symbol = expressionSymbol.getEnclosingScope()
                .resolve( expressionSymbol.getNameToResolveValue(), expressionSymbol.getKind()).get();
        return ((MathValueSymbol)symbol).getValue();
    }
}

