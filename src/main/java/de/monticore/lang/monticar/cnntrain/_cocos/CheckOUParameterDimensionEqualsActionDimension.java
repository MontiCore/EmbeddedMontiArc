/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.MultiParamValueSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants;
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
            final int actionDimension = architectureSymbol.getDimensions().get(outputNameOfTrainedArchitecture).size();

            if (strategyParameters.hasParameter(STRATEGY_OU_MU)) {
                logIfDimensionIsUnequal(configurationSymbol, strategyParameters, outputNameOfTrainedArchitecture,
                        actionDimension, STRATEGY_OU_MU);
            }

            if (strategyParameters.hasParameter(STRATEGY_OU_SIGMA)) {
                logIfDimensionIsUnequal(configurationSymbol, strategyParameters, outputNameOfTrainedArchitecture,
                        actionDimension, STRATEGY_OU_SIGMA);
            }

            if (strategyParameters.hasParameter(STRATEGY_OU_THETA)) {
                logIfDimensionIsUnequal(configurationSymbol, strategyParameters, outputNameOfTrainedArchitecture,
                        actionDimension, STRATEGY_OU_THETA);
            }
        }
    }

    private void logIfDimensionIsUnequal(ConfigurationSymbol configurationSymbol,
                                         MultiParamValueSymbol strategyParameters,
                                         String outputNameOfTrainedArchitecture,
                                         int actionDimension,
                                         String ouParameterName) {
        final int ouParameterDimension = ((Collection<?>) strategyParameters.getParameter(ouParameterName)).size();
        if (ouParameterDimension != actionDimension) {
            Log.error("Vector parameter " + ouParameterName + " of parameter " + STRATEGY_OU + " must have"
                            + " the same dimensions as the action dimension of output "
                            + outputNameOfTrainedArchitecture + " which is " + actionDimension,
                    configurationSymbol.getSourcePosition());
        }
    }
}
