/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import com.google.common.collect.ImmutableSet;
import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Set;

public class CheckDiscreteRLAlgorithmUsesDiscreteStrategy implements CNNTrainASTConfigurationCoCo{
    private static final Set<String> DISCRETE_STRATEGIES = ImmutableSet.<String>builder()
        .add(ConfigEntryNameConstants.STRATEGY_EPSGREEDY)
        .build();

    @Override
    public void check(ASTConfiguration node) {
        if (ASTConfigurationUtils.isDqnAlgorithm(node)
            && ASTConfigurationUtils.hasStrategy(node)
            && ASTConfigurationUtils.getStrategyMethod(node).isPresent()) {
            final String usedStrategy = ASTConfigurationUtils.getStrategyMethod(node).get();
            if (!DISCRETE_STRATEGIES.contains(usedStrategy)) {
                Log.error("0" + ErrorCodes.STRATEGY_NOT_APPLICABLE + " Strategy " + usedStrategy + " used but" +
                 " discrete algorithm used.", node.get_SourcePositionStart());
            }
        }
    }

}
