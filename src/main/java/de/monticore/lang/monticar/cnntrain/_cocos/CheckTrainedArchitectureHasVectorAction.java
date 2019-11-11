/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.List;

public class CheckTrainedArchitectureHasVectorAction implements CNNTrainConfigurationSymbolCoCo {
    @Override
    public void check(ConfigurationSymbol configurationSymbol) {
        if (configurationSymbol.getTrainedArchitecture().isPresent()
                && configurationSymbol.isReinforcementLearningMethod()) {
            final NNArchitectureSymbol trainedArchitecture = configurationSymbol.getTrainedArchitecture().get();
            if (trainedArchitecture.getOutputs().size() == 1) {
                final String actionName = trainedArchitecture.getOutputs().get(0);
                final List<Integer> actionDimensions = trainedArchitecture.getDimensions().get(actionName);
                if (actionDimensions.size() != 1) {
                    Log.error("0" + ErrorCodes.TRAINED_ARCHITECTURE_ERROR
                        + " Output of actor network must be a vector", configurationSymbol.getSourcePosition());
                }
            }
        }
    }
}
