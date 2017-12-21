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
        identifyDefMatrix();
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
     * check which definite matrix has
     * @return JSMatrix with identified definite properties of astMatrix
     */

    private void identifyDefMatrix(){
        double[][] d = getDoubleMatrix();
        RealMatrix matrix = new Array2DRowRealMatrix(d);
        EigenDecomposition e = new EigenDecomposition(matrix);
        if (checkIndef(e)) return;
        checkPosDef(e);
        if (checkPosSemDef(e)) return;
        checkNegDef(e);
        if (checkNegSemDef(e)) return;
        props.add(MatrixProperties.Indef);
    }

    private boolean checkNegSemDef(EigenDecomposition e) {
        boolean flag;
        flag = true;
        for (double v: e.getRealEigenvalues()) {
            if(v > 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.NegSemDef);
            return true;
        }
        return false;
    }

    private void checkNegDef(EigenDecomposition e) {
        boolean flag;
        flag = true;
        for (double v: e.getRealEigenvalues()) {
            if(v >= 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.NegDef);
        }
    }

    private boolean checkPosSemDef(EigenDecomposition e) {
        boolean flag;
        flag = true;
        for (double v: e.getRealEigenvalues()) {
            if(v < 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.PosSemDef);
            return true;
        }
        return false;
    }

    private boolean checkIndef(EigenDecomposition e) {
        if (e.hasComplexEigenvalues()){
            props.add(MatrixProperties.Indef);
            return true;
        }
        return false;
    }

    private void checkPosDef(EigenDecomposition e) {
        boolean flag = true;
        for (double v: e.getRealEigenvalues()) {
            if(v <= 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.PosDef);
        }
    }

    private double[][] getDoubleMatrix() {
        double[][] d= new double[matrix.getRowDimension()][matrix.getColumnDimension()];
        for (int i = 0; i < d.length; i++) {
            for (int j = 0; j < d[0].length; j++) {
                d[i][j] = matrix.getEntry(i,j).getReal();
            }
        }
        return d;
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
        Complex value = getValue(m);
        return value;
    }

    private Complex getValue(Array2DRowFieldMatrix<Complex> m) {
        Complex value = new Complex(0);
        for (int i = 0; i < m.getRowDimension(); i++) {
            Complex sign = new Complex(Math.pow(-1,i));
            Array2DRowFieldMatrix<Complex> m_sub = new Array2DRowFieldMatrix<>(ComplexField.getInstance(), m.getRowDimension() - 1, m.getColumnDimension() - 1);
            goThroughRow(m, i, m_sub);
            value = value.add(sign.multiply(m.getEntry(i,0).multiply(getDeterminant(m_sub))));
        }
        return value;
    }

    private void goThroughRow(Array2DRowFieldMatrix<Complex> m, int i, Array2DRowFieldMatrix<Complex> m_sub) {
        for (int j = 0; j < m.getRowDimension(); j++) {
            if (j < i){
                lowerEntry(m, m_sub, j);
            }
            else if (j > i){
                upperEntry(m, m_sub, j);
            }
        }
    }

    private void upperEntry(Array2DRowFieldMatrix<Complex> m, Array2DRowFieldMatrix<Complex> m_sub, int j) {
        for (int k = 0; k < m.getColumnDimension()-1; k++) {
            m_sub.setEntry(j-1,k,m.getEntry(j,k+1));
        }
    }

    private void lowerEntry(Array2DRowFieldMatrix<Complex> m, Array2DRowFieldMatrix<Complex> m_sub, int j) {
        for (int k = 0; k < m.getColumnDimension()-1; k++) {
            m_sub.setEntry(j,k,m.getEntry(j,k+1));
        }
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

        return getResult((MathArithmeticExpressionSymbol) exp, value1, value2);
    }

    private Complex getResult(MathArithmeticExpressionSymbol exp, Complex value1, Complex value2) {
        Complex res;
        switch (exp.getMathOperator()) {
            case "+": {
                res = value1.add(value2);
                break; }
            case "-": {
                res = value1.subtract(value2);
                break; }
            case "*": {
                res = value1.multiply(value2);
                break; }
            case "/": {
                res = value1.divide(value2);
                break; }
            case "^": {
                res = value1.pow(value2);
                break; }
            default: {
                res = value1;
            } }
        return res;
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
            if (((MathValueExpressionSymbol) expressionSymbol).isNameExpression())
                expressionSymbol = resolveName((MathNameExpressionSymbol) expressionSymbol);
            Complex childExpressionSymbol = getComplex(expressionSymbol);
            if (childExpressionSymbol != null) return childExpressionSymbol;

        }else if (expressionSymbol.isArithmeticExpression())
            return dissolveMathExpression(expressionSymbol);

        return new Complex(0);
    }

    private Complex getComplex(MathExpressionSymbol expressionSymbol) {
        if (expressionSymbol.isParenthesisExpression())
            expressionSymbol = ((MathParenthesisExpressionSymbol)expressionSymbol).getMathExpressionSymbol();

        if (((MathValueExpressionSymbol)expressionSymbol).isNumberExpression())
            return castToComplex((MathNumberExpressionSymbol)expressionSymbol);
        else if (expressionSymbol.isArithmeticExpression())
            return dissolveMathExpression(expressionSymbol);
        return null;
    }

    private MathExpressionSymbol resolveName(MathNameExpressionSymbol expressionSymbol){
        Symbol symbol = expressionSymbol.getEnclosingScope()
                .resolve( expressionSymbol.getNameToResolveValue(), expressionSymbol.getKind()).get();
        return ((MathValueSymbol)symbol).getValue();
    }
}

