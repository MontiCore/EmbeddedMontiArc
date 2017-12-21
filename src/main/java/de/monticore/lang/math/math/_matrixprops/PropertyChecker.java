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
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import java.util.ArrayList;

public class PropertyChecker {
    public static ArrayList<MatrixProperties> checkProps(MathArithmeticExpressionSymbol exp){
        PrologHandler plh = new PrologHandler();
        MathExpressionSymbol leftExpression = getLeftMathExpressionSymbol(exp);
        MathExpressionSymbol rightExpression = getRightMathExpressionSymbol(exp);
        ArrayList<MatrixProperties> props1 = getProps(leftExpression);
        if (props1.isEmpty())
            lookForScalar(((MathNumberExpressionSymbol)leftExpression), "m1", plh);
        String op;
        if (rightExpression.isValueExpression()) {
            if (exp.getMathOperator().equals("^") &&
                    ((MathNumberExpressionSymbol) rightExpression).getValue().getRealNumber().doubleValue() == -1) {
                op = "inv";
                addPrologClauses(plh,props1,"m1");
                return askSolutions(plh, op, false);
            } else op = " " + exp.getMathOperator() + " ";
        } else op = " " + exp.getMathOperator() + " ";
        addPrologClauses(plh,props1,"m1");
        ArrayList<MatrixProperties> props2 = getProps(rightExpression);
        if (props2.isEmpty())
            lookForScalar(((MathNumberExpressionSymbol)rightExpression), "m2", plh);
        addPrologClauses(plh,props2,"m2");
        return askSolutions(plh, op, true);
    }

    private static MathExpressionSymbol getRightMathExpressionSymbol(MathArithmeticExpressionSymbol exp) {
        MathExpressionSymbol rightExpression = exp.getRightExpression();
        if (rightExpression instanceof MathNameExpressionSymbol)
            rightExpression = resolveSymbol(((MathNameExpressionSymbol) rightExpression));
        while (rightExpression.isParenthesisExpression())
            rightExpression = ((MathParenthesisExpressionSymbol)rightExpression).getMathExpressionSymbol();
        return rightExpression;
    }

    private static MathExpressionSymbol getLeftMathExpressionSymbol(MathArithmeticExpressionSymbol exp) {
        MathExpressionSymbol leftExpression = exp.getLeftExpression();
        if (leftExpression instanceof MathNameExpressionSymbol)
            leftExpression = resolveSymbol(((MathNameExpressionSymbol) leftExpression));
        while (leftExpression.isParenthesisExpression())
            leftExpression = ((MathParenthesisExpressionSymbol)leftExpression).getMathExpressionSymbol();
        return leftExpression;
    }

    private static ArrayList<MatrixProperties> askSolutions(PrologHandler plh, String op, boolean binary){
        ArrayList<MatrixProperties> res = new ArrayList<>();
        squareProperty(plh, op, binary, res);
        normalProperty(plh, op, binary, res);
        diagProperty(plh, op, binary, res);
        hermProperty(plh, op, binary, res);
        skewHermProperty(plh, op, binary, res);
        psdProperty(plh, op, binary, res);
        pdProperty(plh, op, binary, res);
        nsdProperty(plh, op, binary, res);
        ndProperty(plh, op, binary, res);
        posProperty(plh, op, binary, res);
        negProperty(plh, op, binary, res);
        invProperty(plh, op, binary, res);
        plh.removeClauses();
        return res;
    }

    private static void invProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "inv(m1,m2,'" + op + "').";
        else query = "inv(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Invertible);
        }
    }

    private static void negProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "neg(m1,m2,'" + op + "').";
        else query = "neg(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Negative);
        }
    }

    private static void posProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "pos(m1,m2,'" + op + "').";
        else query = "pos(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Positive);
        }
    }

    private static void ndProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "nd(m1,m2,'" + op + "').";
        else query = "nd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.NegDef);
        }
    }

    private static void nsdProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "nsd(m1,m2,'" + op + "').";
        else query = "nsd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.NegSemDef);
        }
    }

    private static void pdProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "pd(m1,m2,'" + op + "').";
        else query = "pd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.PosDef);
        }
    }

    private static void psdProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "psd(m1,m2,'" + op + "').";
        else query = "psd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.PosSemDef);
        }
    }

    private static void skewHermProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "skewHerm(m1,m2,'" + op + "').";
        else query = "skewHerm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.SkewHerm);
        }
    }

    private static void hermProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "herm(m1,m2,'" + op + "').";
        else query = "herm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Herm);
        }
    }

    private static void diagProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "diag(m1,m2,'" + op + "').";
        else query = "diag(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Diag);
        }
    }

    private static void normalProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;
        if (binary) query = "norm(m1,m2,'" + op + "').";
        else query = "norm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Norm);
        }
    }

    private static void squareProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        String query;
        if (binary) query = "square(m1,m2,'" + op + "').";
        else query = "square(m1,'" + op + "').";
        ArrayList<String> sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Square);
        }
    }

    private static void lookForScalar(MathNumberExpressionSymbol num, String matrix, PrologHandler plh){
        float f = 1;
        JSValue value = num.getValue();
        f = (value.getRealNumber().floatValue()) % f;
        if (Float.compare(f,0) == 0) {
            createNumber(plh, value, "int(" + matrix + ")", "int+(" + matrix + ")");
        }else{
            createNumber(plh, value, "scal(" + matrix + ")", "scal+(" + matrix + ")");
        }
    }

    private static void createNumber(PrologHandler plh, JSValue value, String str, String str2) {
        plh.addClause(str);
        if (value.getRealNumber().isPositive() || value.getRealNumber().isZero()) plh.addClause(str2);
    }

    private static ArrayList<MatrixProperties> getProps(MathExpressionSymbol sym){
        if(sym instanceof MathMatrixArithmeticValueSymbol)
            return ((MathMatrixArithmeticValueSymbol) sym).getMatrixProperties();
        if (sym.isArithmeticExpression())
            return checkProps((MathArithmeticExpressionSymbol)sym);
        if (sym instanceof MathValueSymbol)
            return ((MathValueSymbol) sym).getMatrixProperties();
        return new ArrayList<>();
    }

    private static void addPrologClauses(PrologHandler plh, ArrayList<MatrixProperties> props, String matrix){
        for (int j = 0; j < props.size(); j++) {
            String str = props.get(j).toString();
            str = str + "(" + matrix + ")";
            plh.addClause(str);
        }
    }

    private static MathExpressionSymbol resolveSymbol(MathNameExpressionSymbol exp){
        String name = exp.getNameToResolveValue();
        SymbolKind kind = exp.getKind();
        Symbol symbol = exp.getEnclosingScope().resolve(name, kind).get();
        return ((MathValueSymbol)symbol).getValue();
    }
}
