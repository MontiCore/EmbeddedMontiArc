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
//package de.monticore.lang.montiarc.math;
//
//import de.monticore.lang.montiarc.math._ast.*;
//import AbstChecker;
//import MathCoCoChecker;
//import MathCocos;
//import de.monticore.lang.montiarc.math._symboltable.*;
//import de.monticore.symboltable.GlobalScope;
//import de.se_rwth.commons.Names;
//import de.se_rwth.commons.logging.Log;
//import org.jscience.mathematics.number.Rational;
//import org.jscience.mathematics.vector.DenseMatrix;
//import org.jscience.mathematics.vector.Matrix;
//
//import java.io.*;
//import java.util.*;
//
///**
// * Created by Tobias PC on 04.01.2017.
// */
//public class Calculator extends AbstCocoCheck{
//
//    public static ASTMathScript calculate(ASTMathScript root) throws IOException{
//        Calculator cal = new Calculator();
//        MathCoCoChecker checker = cal.getChecker();
//        checker.checkAll(root);
//
//        List<ASTMathStatement> statements = root.getMathStatements();
//        ArrayList<MathVariableDeclarationSymbol> symbols = new ArrayList<>();
//        String filename = Names.getQualifiedName(root.getPackage());
//        for(int i = 0; i<statements.size();i++){
//            if(statements.get(i).assignmentIsPresent()){
//                calculate((MathVariableDeclarationSymbol) statements.get(i).getAssignment().get().getSymbol().get());
//            }else if(statements.get(i).conditionalStatementIsPresent()){
//
//            }else if(statements.get(i).forStatementIsPresent()){
//                calculate(statements.get(i).getForStatement().get());
//            }
//        }
//        for(int i = 0; i<statements.size();i++){
//            if(statements.get(i).assignmentIsPresent()){
//                if(!symbols.contains(statements.get(i).getAssignment().get().getSymbol().get())&&
//                        ((MathVariableDeclarationSymbol)statements.get(i).getAssignment().get().getSymbol().get()).getTypeName().isPresent()) {
//                    symbols.add((MathVariableDeclarationSymbol) statements.get(i).getAssignment().get().getSymbol().get());
//                }
//            }else if(statements.get(i).getMathExpression().isPresent()){
//                if(!symbols.contains(statements.get(i).getMathExpression().get().getSymbol().get())) {
//                    symbols.add(calculate((MathVariableDeclarationSymbol) statements.get(i).getMathExpression().get().getSymbol().get()));
//
//                }
//            }
//        }
//
//        writeFile(symbols, filename);
//
//        return cal.loadModel("src/test/resources/Calculations/"+filename+"Sol.m");
//    }
//    public static MathVariableDeclarationSymbol calculate(MathVariableDeclarationSymbol symbol){
//        MathValue val = symbol.getValue();
//        MathVariableDeclarationSymbol tmp = null;
//        if(val instanceof JSValue){
//            return symbol;
//        }else if(val instanceof JSMatrix){
//            ((JSMatrix) val).toMatrix();
//            tmp =  new MathVariableDeclarationSymbol(symbol.getName(),new JSMatrix(((JSMatrix) val).getJScienceMatrix()));
//            tmp.getValue().setUnit(symbol.getValue().getUnit());
//            return  tmp;
//        }else if(val instanceof MathValueReference){
//            Matrix<Rational>test = getRationalMatrix(val);
//            if(test!=null){
//                tmp = new MathVariableDeclarationSymbol(symbol.getName(),new JSMatrix(test));
//                tmp.getValue().setUnit(AbstChecker.getRefUnit((MathValueReference) symbol.getValue()));
//                return tmp;
//            }else{
//                tmp = new MathVariableDeclarationSymbol(symbol.getName(),new JSValue(getRational(val)));
//                tmp.getValue().setUnit(AbstChecker.getRefUnit((MathValueReference) symbol.getValue()));
//                return tmp;
//            }
//        }else if(val instanceof MathExpression){
//            Matrix<Rational>test = getRationalMatrix(val);
//            if(test!=null){
//                tmp = new MathVariableDeclarationSymbol(symbol.getName(),new JSMatrix(test));
//                tmp.getValue().setUnit(AbstChecker.getExpUnit((MathExpression) symbol.getValue()));
//                return tmp;
//            }else{
//                tmp = new MathVariableDeclarationSymbol(symbol.getName(),new JSValue(getRational(val)));
//                tmp.getValue().setUnit(AbstChecker.getExpUnit((MathExpression) symbol.getValue()));
//                return tmp;
//            }
//        }else {
//            Log.error("Unkown Symbol Type");
//            return null;
//        }
//    }
//
//    public static void calculate(ASTForStatement2 forStatement){
//        MathVariableDeclarationSymbol forassignment = (MathVariableDeclarationSymbol) forStatement.getSymbol().get();
//        Matrix<Rational> forIndex = getRationalMatrix(forassignment.getValue());
//        List<ASTMathStatement> statements = forStatement.getMathStatements();
//
//        for(int i = 0; i<forIndex.getNumberOfColumns();i++) {
//            for (int j = 0; j < statements.size(); j++) {
//                if (statements.get(j).assignmentIsPresent()) {
//                    MathVariableDeclarationSymbol assSym = (MathVariableDeclarationSymbol) statements.get(j).getAssignment().get().getSymbol().get();
//                    calculate(assSym);
//                } else if (statements.get(j).conditionalStatementIsPresent()) {
//
//                } else if (statements.get(j).forStatementIsPresent()) {
//                    calculate(statements.get(j).getForStatement().get());
//                } else {
//
//                }
//            }
//            forassignment.setForIndex(Optional.of(forassignment.getForIndex().get() + 1));
//        }
//        forassignment.setForIndex(Optional.of(0));
//    }
//
//    private static void writeFile(ArrayList<MathVariableDeclarationSymbol> symbols,String filename)throws IOException{
//        File test = new File("src/test/resources/Calculations/"+filename+"Sol.m");
//        FileWriter fw = new FileWriter(test);
//        BufferedWriter bw = new BufferedWriter(fw);
//        bw.write("package "+filename+"Sol;");
//        bw.newLine();
//        for(int i = 0; i<symbols.size();i++){
//            bw.write(buildString(symbols.get(i)));
//            bw.newLine();
//        }
//        bw.close();
//    }
//    private static String buildString(MathVariableDeclarationSymbol symbol) {
//        String output="";
//        if(symbol.getTypeName().isPresent()){
//            output += symbol.getTypeName().get();
//            if (symbol.getValue() instanceof JSMatrix) {
//                output += "[" + ((JSMatrix) symbol.getValue()).getJScienceMatrix().getNumberOfRows() + "," + ((JSMatrix) symbol.getValue()).getJScienceMatrix().getNumberOfColumns() + "]";
//            }
//            output += " " + symbol.getName() + " = ";
//        }
//        output+= symbol.getValue().toGrammarExp();
//        if(symbol.getTypeName().isPresent())
//            output+=" ;";
//        return output;
//    }
//
//    public static Matrix<Rational> getRationalMatrix(MathValue value){
//
//        Matrix<Rational> res=null;
//        if(value instanceof JSMatrix){
//            ((JSMatrix)value).toMatrix();
//            return ((JSMatrix) value).getJScienceMatrix();
//        }else if(value instanceof MathExpression) {
//            ArrayList<MathVariableDeclarationSymbol> operands = ((MathExpression) value).getOperands();
//            Operator op = ((MathExpression) value).getOp();
//            //edited
//            if(op.equals(Operator.Assign)){
//                MathVariableDeclarationSymbol op1 = operands.get(0);
//                if(op1.getValue() instanceof MathValueReference){
//                    Matrix<Rational> m = getRationalMatrix(operands.get(1).getValue());
//                    if(m!= null) {
//                        ((MathValueReference) op1.getValue()).getReferencedSymbol().setValue(new JSMatrix(m));
//                        return m;
//                    }else {
//                        return null;
//                    }
//                }
//            }
//
//            if (operands.size() == 1) {
//                if (operands.get(0).getValue() instanceof JSMatrix) {
//                    ((JSMatrix) operands.get(0).getValue()).toMatrix();
//                    if (op.equals(Operator.Trans)) {
//                        return ((JSMatrix) operands.get(0).getValue()).getJScienceMatrix().transpose();
//                    }
//                } else if (operands.get(0).getValue() instanceof MathExpression) {
//                    Matrix<Rational> m = getRationalMatrix(operands.get(0).getValue());
//                    if (op.equals(Operator.Trans)) {
//                        return m.transpose();
//                    }
//                } else if (operands.get(0).getValue() instanceof MathValueReference) {
//                    Matrix<Rational> m = getRationalMatrix(operands.get(0).getValue());
//                    if (op.equals(Operator.Trans)) {
//                        return m.transpose();
//                    }
//                }
//            }
//            for (int i = 0; i < operands.size() - 1; i++) {
//                if (operands.get(i).getValue() instanceof JSMatrix && operands.get(i + 1).getValue() instanceof JSMatrix) {
//                    ((JSMatrix) operands.get(i).getValue()).toMatrix();
//                    ((JSMatrix) operands.get(i + 1).getValue()).toMatrix();
//                    res = doOp(((JSMatrix) operands.get(i).getValue()).getJScienceMatrix(), ((JSMatrix) operands.get(i + 1).getValue()).getJScienceMatrix(), op);
//                } else if (operands.get(i).getValue() instanceof JSMatrix && operands.get(i + 1).getValue() instanceof MathExpression) {
//                    ((JSMatrix) operands.get(i).getValue()).toMatrix();
//                    Matrix<Rational> m = getRationalMatrix((operands.get(i + 1).getValue()));
//                    if (m != null) {
//                        res = doOp(((JSMatrix) operands.get(i).getValue()).getJScienceMatrix(), m, op);
//                    } else {
//                        res = doOp(((JSMatrix) operands.get(i).getValue()).getJScienceMatrix(),getRational(operands.get(i + 1).getValue()), op);
//                    }
//                } else if (operands.get(i).getValue() instanceof MathExpression && operands.get(i + 1).getValue() instanceof JSMatrix) {
//                    ((JSMatrix) operands.get(i + 1).getValue()).toMatrix();
//                    res = doOp(getRationalMatrix(operands.get(i).getValue()), ((JSMatrix) operands.get(i + 1).getValue()).getJScienceMatrix(), op);
//                } else if (operands.get(i).getValue() instanceof MathExpression && operands.get(i + 1).getValue() instanceof MathExpression) {
//                    Matrix<Rational> m = getRationalMatrix(operands.get(i + 1).getValue());
//                    if (m != null) {
//                        res = doOp(getRationalMatrix(operands.get(i).getValue()), m, op);
//                    } else {
//                        res = doOp(getRationalMatrix(operands.get(i).getValue()), getRational(operands.get(i + 1).getValue()), op);
//                    }
//                } else if (operands.get(i).getValue() instanceof JSMatrix && operands.get(i + 1).getValue() instanceof MathValueReference) {
//                    ((JSMatrix) operands.get(i).getValue()).toMatrix();
//                    Matrix<Rational> m = getRationalMatrix(operands.get(i + 1).getValue());
//                    if (m != null) {
//                        res = doOp(((JSMatrix) operands.get(i).getValue()).getJScienceMatrix(), m, op);
//                    } else {
//                        res = doOp(((JSMatrix) operands.get(i).getValue()).getJScienceMatrix(),getRational(operands.get(i + 1).getValue()), op);
//                    }
//                } else if (operands.get(i).getValue() instanceof MathValueReference && operands.get(i + 1).getValue() instanceof JSMatrix) {
//                    ((JSMatrix) operands.get(i + 1).getValue()).toMatrix();
//                    res = doOp(getRationalMatrix(operands.get(i).getValue()), ((JSMatrix) operands.get(i + 1).getValue()).getJScienceMatrix(), op);
//                } else if (operands.get(i).getValue() instanceof MathValueReference && operands.get(i + 1).getValue() instanceof MathExpression) {
//                    Matrix<Rational> m = getRationalMatrix(operands.get(i + 1).getValue());
//                    if (m != null) {
//                        res = doOp(getRationalMatrix(operands.get(i).getValue()), m, op);
//                    } else {
//                        res = doOp(getRationalMatrix(operands.get(i).getValue()),getRational(operands.get(i + 1).getValue()), op);
//                    }
//
//                } else if (operands.get(i).getValue() instanceof MathExpression && operands.get(i + 1).getValue() instanceof MathValueReference) {
//                    Matrix<Rational> m = getRationalMatrix(operands.get(i + 1).getValue());
//                    if (m != null) {
//                        res = doOp(getRationalMatrix(operands.get(i).getValue()), m, op);
//                    } else {
//                        res = doOp(getRationalMatrix(operands.get(i).getValue()),getRational(operands.get(i + 1).getValue()), op);
//                    }
//                } else if (operands.get(i).getValue() instanceof MathValueReference && operands.get(i + 1).getValue() instanceof MathValueReference) {
//                    Matrix<Rational> m = getRationalMatrix(operands.get(i + 1).getValue());
//                    if (m != null) {
//                        res = doOp(getRationalMatrix(operands.get(i).getValue()), m, op);
//                    } else {
//                        res = doOp(getRationalMatrix(operands.get(i).getValue()),getRational(operands.get(i + 1).getValue()), op);
//                    }
//                } else if (operands.get(i).getValue() instanceof JSMatrix && operands.get(i + 1).getValue() instanceof JSValue) {
//                    ((JSMatrix) operands.get(i).getValue()).toMatrix();
//                    res = doOp(((JSMatrix) operands.get(i).getValue()).getJScienceMatrix(), ((JSValue) operands.get(i + 1).getValue()).getNumber(), op);
//                } else if (operands.get(i).getValue() instanceof MathValueReference && operands.get(i + 1).getValue() instanceof JSValue) {
//                    res = doOp(getRationalMatrix(operands.get(i).getValue()), ((JSValue) operands.get(i + 1).getValue()).getNumber(), op);
//                } else if (operands.get(i).getValue() instanceof MathExpression && operands.get(i + 1).getValue() instanceof JSValue) {
//                    res = doOp(getRationalMatrix(operands.get(i).getValue()), ((JSValue) operands.get(i + 1).getValue()).getNumber(), op);
//                }
//            }
//        }else if(value instanceof MathValueReference){
//            if(((MathValueReference) value).getMatrixIndex().isPresent()){
//                return null;
//            }
//            MathVariableDeclarationSymbol ref = ((MathValueReference) value).getReferencedSymbol();
//            MathValue refSym = ref.getValue();
//            if(ref.getForIndex().isPresent()){
//                return null;
//            }
//            if(refSym instanceof JSMatrix){
//                ((JSMatrix) refSym).toMatrix();
//                if(((MathValueReference) value).isEndoperator()){
//                    return endVec(((JSMatrix) refSym).getJScienceMatrix(),((MathValueReference) value).getFromTo());
//                }
//                return ((JSMatrix) refSym).getJScienceMatrix();
//            }else {
//                if(((MathValueReference) value).isEndoperator()){
//                    return endVec(getRationalMatrix(refSym),((MathValueReference) value).getFromTo());
//                }
//                return getRationalMatrix(refSym);
//            }
//        }
//        return res;
//    }
//    private static Matrix<Rational> endVec(Matrix<Rational> matrix,int[][] fromTo){
//        Rational[][]tmp = new Rational[fromTo[0][1]-fromTo[0][0]+1][fromTo[1][1]-fromTo[1][0]+1];
//        for(int i =fromTo[0][0]-1;i<fromTo[0][1];i++){
//            for(int j = fromTo[1][0]-1;j<fromTo[1][1];j++){
//                tmp[i][j]=matrix.get(i,j);
//            }
//        }
//        return DenseMatrix.valueOf(tmp);
//    }
//    public static Rational getRational(MathValue value){
//        Rational res = Rational.ZERO;
//        if(value instanceof JSValue){
//            return ((JSValue) value).getNumber();
//        }else if (value instanceof MathExpression) {
//            ArrayList<MathVariableDeclarationSymbol> operands = ((MathExpression) value).getOperands();
//            Operator op = ((MathExpression) value).getOp();
//            //edited
//            if(op.equals(Operator.Assign)){
//                MathVariableDeclarationSymbol op1 = operands.get(0);
//                if(op1.getValue() instanceof MathValueReference){
//                    Rational r = getRational(operands.get(1).getValue());
//                    ((MathValueReference) op1.getValue()).getReferencedSymbol().setValue(new JSValue(r));
//                    return  r;
//                }
//            }
//            for (int i = 0; i < operands.size() - 1; i++) {
//                if (operands.get(i).getValue() instanceof JSValue && operands.get(i + 1).getValue() instanceof JSValue) {
//                    res = doOp(((JSValue) operands.get(i).getValue()).getNumber(), ((JSValue) operands.get(i + 1).getValue()).getNumber(), op);
//                } else if (operands.get(i).getValue() instanceof JSValue && operands.get(i + 1).getValue() instanceof MathExpression) {
//                    res = doOp(((JSValue) operands.get(i).getValue()).getNumber(), getRational(operands.get(i + 1).getValue()), op);
//                } else if (operands.get(i).getValue() instanceof MathExpression && operands.get(i + 1).getValue() instanceof JSValue) {
//                    res = doOp(getRational(operands.get(i).getValue()), ((JSValue) operands.get(i + 1).getValue()).getNumber(), op);
//                } else if (operands.get(i).getValue() instanceof MathExpression && operands.get(i + 1).getValue() instanceof MathExpression) {
//                    res = doOp(getRational(operands.get(i).getValue()),getRational(operands.get(i + 1).getValue()), op);
//                } else if (operands.get(i).getValue() instanceof JSValue && operands.get(i + 1).getValue() instanceof MathValueReference) {
//                    res = doOp(((JSValue) operands.get(i).getValue()).getNumber(), getRational(operands.get(i + 1).getValue()), op);
//                } else if (operands.get(i).getValue() instanceof MathValueReference && operands.get(i + 1).getValue() instanceof JSValue) {
//                    res = doOp(getRational(operands.get(i).getValue()), ((JSValue) operands.get(i + 1).getValue()).getNumber(), op);
//                } else if (operands.get(i).getValue() instanceof MathValueReference && operands.get(i + 1).getValue() instanceof MathExpression) {
//                    res = doOp(getRational(operands.get(i).getValue()), getRational(operands.get(i + 1).getValue()), op);
//                } else if (operands.get(i).getValue() instanceof MathExpression && operands.get(i + 1).getValue() instanceof MathValueReference) {
//                    res = doOp(getRational(operands.get(i).getValue()),getRational(operands.get(i + 1).getValue()), op);
//                } else {
//                    res = doOp(getRational(operands.get(i).getValue()),getRational(operands.get(i + 1).getValue()), op);
//                }
//            }
//        }else if(value instanceof MathValueReference){
//            MathVariableDeclarationSymbol ref = ((MathValueReference) value).getReferencedSymbol();
//            MathValue refSym = ref.getValue();
//            if(ref.getForIndex().isPresent()) {
//                Matrix<Rational> m = getRationalMatrix(refSym);
//                return m.get(0,ref.getForIndex().get());
//            }
//            if(refSym instanceof JSValue){
//                return ((JSValue) refSym).getNumber();
//            }else if(refSym instanceof MathExpression){
//                return getRational(refSym);
//            }else if(refSym instanceof JSMatrix){
//                if(((MathValueReference) value).getMatrixIndex().isPresent()){
//                    Matrix<Rational> m = getRationalMatrix(((MathValueReference) value).getMatrixIndex().get());
//                    if(m.getNumberOfColumns()==1) {
//                        return getRational(((JSMatrix) refSym).getElement(0, m.get(0, 0).intValue()));
//                    }else {
//                        return getRational(((JSMatrix) refSym).getElement(m.get(0, 0).intValue(),m.get(0,1).intValue()));
//                    }
//                }
//            }else {
//                return getRational(refSym);
//            }
//        }
//        return res;
//    }
//
//    private static Rational doOp(Rational r1,Rational r2,Operator op){
//        switch (op){
//            case Plus: return r1.plus(r2);
//            case Minus:return r1.minus(r2);
//            case Times:return r1.times(r2);
//            case Slash:return r1.divide(r2);
//            case Div: return r1.divide(r2);
//            case Mod: return Rational.valueOf(r1.intValue()%r2.intValue(),1);
//            case Power:return r1.pow(r2.intValue());
//            default:
//                Log.error("Keine gültige Operation");
//                return Rational.ONE;
//        }
//    }
//    private static Matrix<Rational> doOp(Matrix<Rational> r1,Matrix<Rational> r2,Operator op){
//        if(r1==null||r2==null){
//            return null;
//        }
//        switch (op){
//            case Plus: return r1.plus(r2);
//            case Minus:return r1.minus(r2);
//            case Times:return r1.times(r2);
//            case TimesWise:return timeWise(r1,r2);
//            case SolEqu:return r1.solve(r2);
//            default:
//                Log.error("Keine gültige Operation");
//                return r1;
//        }
//    }
//    private static Matrix<Rational> doOp(Matrix<Rational> r1, Rational r2, Operator op){
//        if(op.equals(Operator.Power)){
//            return r1.pow(r2.intValue());
//        }else if(op.equals(Operator.PowerWise)){
//            return powerWise(r1,r2);
//        }
//        return r1;
//    }
//
//    private static Matrix<Rational> timeWise(Matrix<Rational> r1,Matrix<Rational> r2){
//        Rational[][] res = new Rational[r1.getNumberOfRows()][r1.getNumberOfColumns()];
//        for(int i =0; i<r1.getNumberOfRows();i++){
//            for(int j=0; j<r1.getNumberOfColumns();j++){
//                res[i][j]= r1.get(i,j).times(r2.get(i,j));
//            }
//        }
//        return DenseMatrix.valueOf(res);
//    }
//    private static Matrix<Rational> powerWise(Matrix<Rational> r1,Rational r2){
//        Rational[][] res = new Rational[r1.getNumberOfRows()][r1.getNumberOfColumns()];
//        for(int i =0; i<r1.getNumberOfRows();i++){
//            for(int j=0; j<r1.getNumberOfColumns();j++){
//                res[i][j]= r1.get(i,j).pow(r2.intValue());
//            }
//        }
//        return DenseMatrix.valueOf(res);
//    }
//
//    @Override
//    protected MathCoCoChecker getChecker() {
//        return MathCocos.createChecker();
//    }
//}
