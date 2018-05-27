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

import de.monticore.lang.math._symboltable.JSValue;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import java.util.ArrayList;

public class PropertyChecker {
    public static ArrayList<MatrixProperties> checkProps(IArithmeticExpression expressionSymbol){
        PrologHandler plh = new PrologHandler();
        MathExpressionSymbol leftExpression = getChildMathExpressionSymbol(expressionSymbol.getLeftExpression());
        MathExpressionSymbol rightExpression = getChildMathExpressionSymbol(expressionSymbol.getRightExpression());
        ArrayList<MatrixProperties> props1 = getProps(leftExpression);
        if (props1.isEmpty())
            lookForScalar(((MathNumberExpressionSymbol)leftExpression), "m1", plh);
        String op = "inv";
        if (getPropertiesOfInv(expressionSymbol, plh, rightExpression, props1, op)){
            AskSolution sol = new AskSolution(plh,op,false);
            return sol.askSolutions();
        }
        op = " " + expressionSymbol.getOperator() + " ";
        addPrologClauses(plh,props1,"m1");
        ArrayList<MatrixProperties> props2 = getProps(rightExpression);
        if (props2.isEmpty())
            lookForScalar(((MathNumberExpressionSymbol)rightExpression), "m2", plh);
        addPrologClauses(plh,props2,"m2");
        AskSolution sol = new AskSolution(plh,op, true);
        return sol.askSolutions();
    }

    private static boolean getPropertiesOfInv(IArithmeticExpression exp, PrologHandler plh, MathExpressionSymbol rightExpression, ArrayList<MatrixProperties> props1, String op) {
        if (rightExpression.isValueExpression()) {
            if (exp.getOperator().equals("^") &&
                    ((MathNumberExpressionSymbol) rightExpression).getValue().getRealNumber().doubleValue() == -1) {
                addPrologClauses(plh,props1,"m1");
                return true;
            }
        }
        return false;
    }

    private static MathExpressionSymbol getChildMathExpressionSymbol(MathExpressionSymbol exp) {
        if (exp instanceof MathNameExpressionSymbol)
            exp = resolveSymbol(((MathNameExpressionSymbol) exp));
        while (exp.isParenthesisExpression())
            exp = ((MathParenthesisExpressionSymbol)exp).getMathExpressionSymbol();
        return exp;
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
        if (sym.isArithmeticExpression() || sym instanceof MathMatrixArithmeticExpressionSymbol)
            return checkProps((IArithmeticExpression) sym);
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
