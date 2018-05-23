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
package de.monticore.lang.math.legacy;

import de.monticore.assignmentexpressions._ast.ASTAssignmentExpression;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._ast.*;
import de.se_rwth.commons.logging.Log;

public class AssignmentExpressionHelper {
    private static AssignmentExpressionHelper ourInstance = new AssignmentExpressionHelper();

    public static AssignmentExpressionHelper getInstance() {
        return ourInstance;
    }

    private AssignmentExpressionHelper() {
    }

    public ASTMathAssignmentExpression convert(ASTAssignmentExpression regular) {
        // create new ast node
        ASTMathAssignmentExpression math = MathMill.mathAssignmentExpressionBuilder().build();
        // set empty
        setLeftAbsent(math);
        // set left
        ASTExpression left = regular.getLeftExpression();
        if (left instanceof ASTNameExpression)
            math.setName(((ASTNameExpression) left).getName());
        else if (left instanceof ASTMathMatrixNameExpression)
            math.setMathMatrixNameExpression((ASTMathMatrixNameExpression) left);
        else if (left instanceof ASTMathDottedNameExpression)
            math.setMathDottedNameExpression((ASTMathDottedNameExpression) left);
        else
            Log.error(String.format("Invalid left part of assignment expression: %s", regular.getLeftExpression().toString()), regular.getLeftExpression().get_SourcePositionStart());
        // set operator
        math.setMathAssignmentOperator(getOperator(regular));
        // set right
        math.setExpression(regular.getRightExpression());
        // assign symbols
        setSymbols(regular, math);
        return math;
    }

    public ASTMathAssignmentOperator getOperator(ASTAssignmentExpression regular) {
        ASTMathAssignmentOperator operator = MathMill.mathAssignmentOperatorBuilder().build();
        operator.setOperator(regular.getOperator());
        return operator;
    }

    public void setSymbols(ASTExpression from, ASTExpression to) {
        to.setEnclosingScopeOpt(from.getEnclosingScopeOpt());
        to.setSymbolOpt(from.getSymbolOpt());
    }

    protected void setLeftAbsent(ASTMathAssignmentExpression math) {
        math.setNameAbsent();
        math.setMathMatrixNameExpressionAbsent();
        math.setMathDottedNameExpressionAbsent();
    }
}
