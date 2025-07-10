/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import static de.monticore.lang.monticar.cnntrain._cocos.ASTConfigurationUtils.*;

public class CheckRosEnvironmentHasOnlyOneRewardSpecification implements CNNTrainASTConfigurationCoCo {
    @Override
    public void check(final ASTConfiguration node) {
        if (isReinforcementLearning(node)
                && hasRosEnvironment(node)
                && hasRewardFunction(node)
                && hasRewardTopic(node)) {
                Log.error("0" + ErrorCodes.CONTRADICTING_PARAMETERS
                        + " Multiple reward calculation method specified. Either use a reward function component with "
                        + "parameter reward_function or use ROS topic with parameter reward_topic. "
                        + "Both is not possible");
        }
    }
}
