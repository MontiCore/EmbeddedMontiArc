/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._cocos;

import de.monticore.lang.math._ast.ASTMathAssignmentDeclarationStatement;
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
 */
public class OptimizationStatementCheck implements MathOptASTOptimizationStatementCoCo {

    private Set<String> supportedReturnTypes = new HashSet<>(Arrays.asList("Q", "Z"));
    private Set<String> supportedOptimizationTypes = new HashSet<>(Arrays.asList("Q", "Z"));

    @Override
    public void check(ASTOptimizationStatement node) {
        checkObjectiveFunction(node);
        checkObjectiveFunctionReturnVariable(node);
        checkOptimizationVariable(node);
        checkIndependentVariables(node);
        checkStepSize(node);
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
        for (ASTOptimizationVariableDeclaration astOptVar :node.getOptimizationVariableList()) {
            if (!supportedOptimizationTypes.contains(astOptVar.getType().getElementType().getName())) {
                Log.error(String.format("0xC0003 Optimization variable type \"%s\" is not supported as return value.", astOptVar.getType().toString()));
            }
        }
    }

    private void checkIndependentVariables(ASTOptimizationStatement node) {
        for (ASTMathAssignmentDeclarationStatement indVar :node.getIndependentDeclarationList()) {
            if (!supportedOptimizationTypes.contains(indVar.getType().getElementType().getName())) {
                Log.error(String.format("0xC0004 Optimization variable type \"%s\" is not supported as return value.", indVar.getType().toString()));
            }
        }
    }

    private void checkStepSize(ASTOptimizationStatement node) {
        if(node.getStepSizeOpt().isPresent()){
            //
        }
    }

    //ToDo: Check for head loop

}
