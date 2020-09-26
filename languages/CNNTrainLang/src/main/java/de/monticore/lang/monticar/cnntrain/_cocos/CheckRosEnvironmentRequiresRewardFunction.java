/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import static de.monticore.lang.monticar.cnntrain._cocos.ASTConfigurationUtils.*;

public class CheckRosEnvironmentRequiresRewardFunction implements CNNTrainASTConfigurationCoCo {
    @Override
    public void check(final ASTConfiguration node) {
        // Specification of reward function only required for reinforcement learning via ROS since OpenAI Gym defines
        // their own reward functions
        if (isReinforcementLearning(node) && hasRosEnvironment(node)) {
            // Reward needs to be either be calculated with a custom component or
            if (!hasRewardFunction(node) && !hasRewardTopic(node)) {
                Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING
                        + " Reward function is missing. Either add a reward function component with parameter "
                        + "reward_function or add a ROS topic with parameter reward_topic.");
            }
        }
    }
}
