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

import static de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants.*;

public class CheckContinuousRLAlgorithmUsesContinuousStrategy implements CNNTrainASTConfigurationCoCo{
    private static final Set<String> CONTINUOUS_STRATEGIES = ImmutableSet.<String>builder()
        .add(STRATEGY_OU)
        .add(STRATEGY_GAUSSIAN)
        .build();

    @Override
    public void check(ASTConfiguration node) {
        if (ASTConfigurationUtils.isContinuousAlgorithm(node)
            && ASTConfigurationUtils.hasStrategy(node)
            && ASTConfigurationUtils.getStrategyMethod(node).isPresent()) {
            final String usedStrategy = ASTConfigurationUtils.getStrategyMethod(node).get();
            if (!CONTINUOUS_STRATEGIES.contains(usedStrategy)) {
                Log.error("0" + ErrorCodes.STRATEGY_NOT_APPLICABLE + " Strategy " + usedStrategy + " used but" +
                 " continuous algorithm used.", node.get_SourcePositionStart());
            }
        }
    }
}
