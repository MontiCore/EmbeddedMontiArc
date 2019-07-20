/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
