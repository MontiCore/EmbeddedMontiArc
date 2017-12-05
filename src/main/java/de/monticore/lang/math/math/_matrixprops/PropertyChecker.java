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

        MathExpressionSymbol leftExpression = exp.getLeftExpression();
        MathExpressionSymbol rightExpression = exp.getRightExpression();

        if (leftExpression instanceof MathNameExpressionSymbol)
            leftExpression = resolveSymbol(((MathNameExpressionSymbol) leftExpression));

        while (leftExpression.isParenthesisExpression())
            leftExpression = ((MathParenthesisExpressionSymbol)leftExpression).getMathExpressionSymbol();

        if (rightExpression instanceof MathNameExpressionSymbol)
            rightExpression = resolveSymbol(((MathNameExpressionSymbol) rightExpression));

        while (rightExpression.isParenthesisExpression())
            rightExpression = ((MathParenthesisExpressionSymbol)rightExpression).getMathExpressionSymbol();

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

    public static void changeAssignmentProps(MathAssignmentExpressionSymbol assignmentExpressionSymbol, Symbol assigned){
        if (((MathValueSymbol)assigned).isValueExpression()){
            MathExpressionSymbol expressionSymbol = assignmentExpressionSymbol.getExpressionSymbol();
            ArrayList<MatrixProperties> props = new ArrayList<>();

            while (expressionSymbol.isParenthesisExpression())
                ((MathParenthesisExpressionSymbol)expressionSymbol).getMathExpressionSymbol();


            MathArithmeticExpressionSymbol arithmeticExpressionSymbol = new MathArithmeticExpressionSymbol();
            arithmeticExpressionSymbol.setLeftExpression(((MathExpressionSymbol)assigned));
            arithmeticExpressionSymbol.setRightExpression(expressionSymbol);
            switch (assignmentExpressionSymbol.getAssignmentOperator().getOperator()) {
                case "=":{
                    if (expressionSymbol instanceof MathArithmeticExpressionSymbol)
                        props = checkProps(((MathArithmeticExpressionSymbol) expressionSymbol));
                    else if (expressionSymbol instanceof MathMatrixArithmeticValueSymbol)
                        props = ((MathMatrixArithmeticValueSymbol) expressionSymbol).getMatrixProperties();
                    break;
                }
                case "+=": {
                    arithmeticExpressionSymbol.setMathOperator("+");
                    props = checkProps(arithmeticExpressionSymbol);
                    break;
                }
                case "-=": {
                    arithmeticExpressionSymbol.setMathOperator("-");
                    props = checkProps(arithmeticExpressionSymbol);
                    break;
                }
                case "*=": {
                    arithmeticExpressionSymbol.setMathOperator("*");
                    props = checkProps(arithmeticExpressionSymbol);
                    break;
                }
            }

            ArrayList<String> props_String = new ArrayList<>();
            for (int i = 0; i < props.size(); i++) props_String.add(props.get(i).toString());

            ((MathValueSymbol) assigned).getType().setProperties(props_String);
        }
    }

    private static ArrayList<MatrixProperties> askSolutions(PrologHandler plh, String op, boolean binary){
        ArrayList<MatrixProperties> res = new ArrayList<>();

        String query;
        if (binary) query = "square(m1,m2,'" + op + "').";
        else query = "square(m1,'" + op + "').";
        ArrayList<String> sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Square);
        }

        if (binary) query = "norm(m1,m2,'" + op + "').";
        else query = "norm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Norm);
        }

        if (binary) query = "diag(m1,m2,'" + op + "').";
        else query = "diag(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Diag);
        }

        if (binary) query = "herm(m1,m2,'" + op + "').";
        else query = "herm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Herm);
        }

        if (binary) query = "skewHerm(m1,m2,'" + op + "').";
        else query = "skewHerm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.SkewHerm);
        }

        if (binary) query = "psd(m1,m2,'" + op + "').";
        else query = "psd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.PosSemDef);
        }

        if (binary) query = "pd(m1,m2,'" + op + "').";
        else query = "pd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.PosDef);
        }

        if (binary) query = "nsd(m1,m2,'" + op + "').";
        else query = "nsd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.NegSemDef);
        }

        if (binary) query = "nd(m1,m2,'" + op + "').";
        else query = "nd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.NegDef);
        }
        plh.removeClauses();

        return res;
    }

    private static void lookForScalar(MathNumberExpressionSymbol num, String matrix, PrologHandler plh){
        float f = 1;
        JSValue value = num.getValue();
        f = (value.getRealNumber().floatValue()) % f;
        if (Float.compare(f,0) == 0) {
            plh.addClause("int(" + matrix + ")");
            if (value.getRealNumber().isPositive() || value.getRealNumber().isZero()) plh.addClause("int+(" + matrix + ")");
        }else{
            plh.addClause("scal(" + matrix + ")");
            if (value.getRealNumber().isPositive() || value.getRealNumber().isZero()) plh.addClause("scal+(" + matrix + ")");
        }
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
