package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.RLAlgorithm;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

/**
 *
 */
public class CheckTrainedRlNetworkHasExactlyOneOutput implements CNNTrainConfigurationSymbolCoCo {
    @Override
    public void check(final ConfigurationSymbol configurationSymbol) {
        if (configurationSymbol.isReinforcementLearningMethod()
                && configurationSymbol.getTrainedArchitecture().isPresent()) {
            final int numberOfOutputs = configurationSymbol.getTrainedArchitecture().get().getOutputs().size();
            if (numberOfOutputs != 1) {
                final String networkName
                        = configurationSymbol.getEntry("rl_algorithm").getValue().getValue()
                        .equals(RLAlgorithm.DQN) ? "Q-Network" : "Actor-Network";
                Log.error("x0" + ErrorCodes.TRAINED_ARCHITECTURE_ERROR
                        + networkName + " " +configurationSymbol.getTrainedArchitecture().get().getName()
                        +" has " + numberOfOutputs + " outputs but 1 is only allowed.", configurationSymbol.getSourcePosition());
            }
        }
    }
}