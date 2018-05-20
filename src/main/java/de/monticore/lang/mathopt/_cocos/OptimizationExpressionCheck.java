/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
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
package de.monticore.lang.mathopt._cocos;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.mathopt._ast.ASTOptimizationExpression;

/**
 * Context Conditions for optimization statement
 *
 * @author Christoph Richter
 */
public class OptimizationExpressionCheck implements MathOptASTOptimizationExpressionCoCo {


    @Override
    public void check(ASTOptimizationExpression node) {
        checkObjectiveFunctionReturnVariable(node.getObjectiveFunction());
    }

    /**
     * Checks if the return value of the objective function is scalar
     *
     * @param objFunc AST objective function expression
     */
    private void checkObjectiveFunctionReturnVariable(ASTExpression objFunc) {
        // TODO check return variable dimension
    }

}
