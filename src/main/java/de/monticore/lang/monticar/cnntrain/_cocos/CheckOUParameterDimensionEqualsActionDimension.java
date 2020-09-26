/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.MultiParamValueSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.List;

import static de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants.*;

/**
 *
 */
public class CheckOUParameterDimensionEqualsActionDimension implements CNNTrainConfigurationSymbolCoCo {
    @Override
    public void check(final ConfigurationSymbol configurationSymbol) {
        if (configurationSymbol.getTrainedArchitecture().isPresent()
                && configurationSymbol.isReinforcementLearningMethod()
                && configurationSymbol.getEntry(STRATEGY).getValue().getValue().equals(STRATEGY_OU)) {
            final MultiParamValueSymbol strategyParameters
                    = (MultiParamValueSymbol)configurationSymbol.getEntry(STRATEGY).getValue();
            final NNArchitectureSymbol architectureSymbol = configurationSymbol.getTrainedArchitecture().get();
            final String outputNameOfTrainedArchitecture = architectureSymbol.getOutputs().get(0);
            final List<Integer> actionDimensions = architectureSymbol.getDimensions().get(outputNameOfTrainedArchitecture);
            assert actionDimensions.size() == 1: "Invalid action: DDPG Actor model requires action to be a vector";
            final int vectorSize = actionDimensions.get(0);

            if (strategyParameters.hasParameter(STRATEGY_OU_MU)) {
                logIfDimensionIsUnequal(configurationSymbol, strategyParameters, outputNameOfTrainedArchitecture,
                        vectorSize, STRATEGY_OU_MU);
            }

            if (strategyParameters.hasParameter(STRATEGY_OU_SIGMA)) {
                logIfDimensionIsUnequal(configurationSymbol, strategyParameters, outputNameOfTrainedArchitecture,
                        vectorSize, STRATEGY_OU_SIGMA);
            }

            if (strategyParameters.hasParameter(STRATEGY_OU_THETA)) {
                logIfDimensionIsUnequal(configurationSymbol, strategyParameters, outputNameOfTrainedArchitecture,
                        vectorSize, STRATEGY_OU_THETA);
            }
        }
    }

    private void logIfDimensionIsUnequal(ConfigurationSymbol configurationSymbol,
                                         MultiParamValueSymbol strategyParameters,
                                         String outputNameOfTrainedArchitecture,
                                         int actionVectorDimension,
                                         String ouParameterName) {
        final int ouParameterDimension = ((Collection<?>) strategyParameters.getParameter(ouParameterName)).size();
        if (ouParameterDimension != actionVectorDimension) {
            Log.error("0" + ErrorCodes.TRAINED_ARCHITECTURE_ERROR
                            + " Vector parameter " + ouParameterName + " of parameter " + STRATEGY_OU + " must have"
                            + " the same dimensions as the action dimension of output "
                            + outputNameOfTrainedArchitecture + " which is " + actionVectorDimension,
                    configurationSymbol.getSourcePosition());
        }
    }
}
