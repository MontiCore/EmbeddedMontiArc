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
package de.monticore.lang.math.math._cocos;


import de.monticore.lang.math.math._ast.ASTMathAssignmentExpression;
import de.monticore.lang.math.math._matrixprops.MatrixProperties;
import de.monticore.lang.math.math._matrixprops.PropertyChecker;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Philipp Goerick
 *
 * Matrix Properties Coco
 */

public class MatrixAssignmentCheck implements MathASTMathAssignmentExpressionCoCo {
    @Override
    public void check(ASTMathAssignmentExpression assignment) {
        Symbol symbol = assignment.getEnclosingScope().get()
                .resolve(assignment.getName().get(),new MathExpressionSymbolKind()).get();
        if (!((MathValueSymbol)symbol).getType().getProperties().isEmpty()) {
            checkMatrixOperation(assignment, (MathValueSymbol) symbol);
        }
    }

    private void checkMatrixOperation(ASTMathAssignmentExpression assignment, MathValueSymbol symbol) {
        MathExpressionSymbol expressionSymbol = ((MathExpressionSymbol)assignment.getMathExpression().getSymbol().get());
        List<String> expProps = symbol.getType().getProperties();
        ArrayList<MatrixProperties> props = new ArrayList<>();
        MathArithmeticExpressionSymbol arithmeticExpressionSymbol = new MathArithmeticExpressionSymbol();
        arithmeticExpressionSymbol.setLeftExpression(symbol.getValue());
        arithmeticExpressionSymbol.setRightExpression(expressionSymbol);
        props = getMatrixProperties(assignment, expressionSymbol, props, arithmeticExpressionSymbol);
        compareArrays(symbol, expProps, props);
    }

    private ArrayList<MatrixProperties> getMatrixProperties(ASTMathAssignmentExpression assignment, MathExpressionSymbol expressionSymbol, ArrayList<MatrixProperties> props, MathArithmeticExpressionSymbol arithmeticExpressionSymbol) {
        switch (assignment.getMathAssignmentOperator().getOperator().get()) {
            case "=":{
                props = solveEquation(assignment, expressionSymbol, props);
                break; }
            case "+=": {
                arithmeticExpressionSymbol.setMathOperator("+");
                props = PropertyChecker.checkProps(arithmeticExpressionSymbol);
                break; }
            case "-=": {
                arithmeticExpressionSymbol.setMathOperator("-");
                props = PropertyChecker.checkProps(arithmeticExpressionSymbol);
                break; }
            case "*=": {
                arithmeticExpressionSymbol.setMathOperator("*");
                props = PropertyChecker.checkProps(arithmeticExpressionSymbol);
                break; }
        }
        return props;
    }

    private ArrayList<MatrixProperties> solveEquation(ASTMathAssignmentExpression assignment, MathExpressionSymbol expressionSymbol, ArrayList<MatrixProperties> props) {
        if (expressionSymbol instanceof IArithmeticExpression)
            props = PropertyChecker.checkProps(((IArithmeticExpression) expressionSymbol));
        else if (expressionSymbol instanceof MathMatrixArithmeticValueSymbol)
            props = ((MathMatrixArithmeticValueSymbol) expressionSymbol).getMatrixProperties();
        else if (expressionSymbol instanceof MathNameExpressionSymbol){
            Symbol sym = assignment.getEnclosingScope().get()
                    .resolve(((MathNameExpressionSymbol) expressionSymbol).getNameToResolveValue(),new MathExpressionSymbolKind()).get();
            props = ((MathValueSymbol) sym).getMatrixProperties();
        }
        return props;
    }

    private void compareArrays(MathValueSymbol symbol, List<String> expProps, ArrayList<MatrixProperties> props) {
        ArrayList<String> props_String = new ArrayList<>();
        for (int i = 0; i < props.size(); i++) props_String.add(props.get(i).toString());

        if (!props_String.containsAll(expProps))
            Log.error("Matrix does not fullfill given properties");
        else{
            symbol.setMatrixProperties(props);
        }
    }
}
