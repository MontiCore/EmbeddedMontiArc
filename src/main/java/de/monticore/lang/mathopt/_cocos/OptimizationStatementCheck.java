/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * <p>
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * <p>
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.mathopt._cocos;

import de.monticore.lang.mathopt._ast.ASTOptimizationObjectiveValue;
import de.monticore.lang.mathopt._ast.ASTOptimizationStatement;
import de.monticore.lang.mathopt._ast.ASTOptimizationVariableDeclaration;
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

    private Set<String> supportedReturnTypes = new HashSet<>(Arrays.asList("Q, Z"));
    private Set<String> supportedOptimizationTypes = new HashSet<>(Arrays.asList("Q, Z"));

    @Override
    public void check(ASTOptimizationStatement node) {
        checkObjectiveFunction(node);
        checkObjectiveFunctionReturnVariable(node);
        checkOptimizationVariable(node);
    }

    private void checkObjectiveFunction(ASTOptimizationStatement node) {
        // 0xC0001
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
                Log.error(String.format("0xC0002 Objective value type \"%s\" is not supported as return value.", astObjectiveValue.getType().getName()));
            }
        }
    }

    private void checkOptimizationVariable(ASTOptimizationStatement node) {
        ASTOptimizationVariableDeclaration astOptVar = node.getOptimizationVariable();
        if (!supportedOptimizationTypes.contains(astOptVar.getType().getElementType())) {
            Log.error(String.format("0xC0003 Optimization variable type \"%s\" is not supported as return value.", astOptVar.getType().toString()));
        }
    }


}
