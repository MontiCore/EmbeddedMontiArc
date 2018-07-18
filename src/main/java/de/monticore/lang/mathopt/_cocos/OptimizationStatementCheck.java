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
import de.monticore.lang.mathopt._ast.ASTOptimizationStatement;
import de.monticore.lang.mathopt._ast.ASTOptimizationObjectiveValue;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * Context Conditions for optimization statement
 *
 * @author Christoph Richter
 */
public class OptimizationStatementCheck implements MathOptASTOptimizationStatementCoCo {

    private Set<String> supportedReturnTypes = new HashSet<>(Arrays.asList("Q"));

    @Override
    public void check(ASTOptimizationStatement node) {
        checkObjectiveFunctionReturnVariable(node);
    }

    /**
     * Checks if the return value of the objective function has the correct type
     *
     * @param node ASTOptimizationStatement
     */
    private void checkObjectiveFunctionReturnVariable(ASTOptimizationStatement node) {
        if (node.getObjectiveValueOpt().isPresent()) {
            ASTOptimizationObjectiveValue astObjectiveValue = node.getObjectiveValueOpt().get();
            if (!supportedReturnTypes.contains(astObjectiveValue.getType().getName())) {
                Log.error(String.format("0x8E75E9 Objective value type \"%s\" is not supported as return value.", astObjectiveValue.getType().getName()));
            }
        }
    }

}
