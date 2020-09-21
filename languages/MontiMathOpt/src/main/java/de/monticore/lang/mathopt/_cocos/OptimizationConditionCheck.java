/* (c) https://github.com/MontiCore/monticore */
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
