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

import de.monticore.lang.mathopt._ast.ASTOptimizationBoundsCondition;
import de.monticore.lang.mathopt._ast.ASTOptimizationCondition;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * Context Conditions for optimization statement
 *
 * @author Christoph Richter
 */
public class OptimizationConditionCheck implements MathOptASTOptimizationConditionCoCo {

    @Override
    public void check(ASTOptimizationCondition node) {
        checkBoxConstraint(node);
    }

    /**
     * Checks if the return value of the objective function has the correct type
     *
     * @param node ASTOptimizationStatement
     */
    private void checkBoxConstraint(ASTOptimizationCondition node) {
        if (node.getBoundedConditionOpt().isPresent()) {
            ASTOptimizationBoundsCondition astCondition = node.getBoundedConditionOpt().get();
            if ((astCondition.getLower() instanceof ASTNumberWithUnit) && (astCondition.getUpper() instanceof ASTNumberWithUnit)) {
                Double lowerVal = ((ASTNumberWithUnit) astCondition.getLower()).getNumber().orElse(-1E30);
                Double upperVal = ((ASTNumberWithUnit) astCondition.getUpper()).getNumber().orElse(1E30);
                if (upperVal < lowerVal) {
                    Log.warn("0xC0004 Lower bound of constraint is greater than the upper bound.", astCondition.get_SourcePositionStart());
                }
            }
        }
    }

}
