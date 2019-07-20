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
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.annotations.Range;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.stream.Collectors;

/**
 *
 */
public class CheckCriticNetworkInputs implements CNNTrainConfigurationSymbolCoCo {

    @Override
    public void check(ConfigurationSymbol configurationSymbol) {
        if (configurationSymbol.getCriticNetwork().isPresent()) {
            if (!configurationSymbol.getTrainedArchitecture().isPresent()) {
                Log.error("0" + ErrorCodes.MISSING_TRAINED_ARCHITECTURE +
                        "No architecture found that is trained by this configuration.", configurationSymbol.getSourcePosition());
            }
            NNArchitectureSymbol trainedArchitecture = configurationSymbol.getTrainedArchitecture().get();
            NNArchitectureSymbol criticNetwork = configurationSymbol.getCriticNetwork().get();

            if (trainedArchitecture.getInputs().size() != 1 || trainedArchitecture.getOutputs().size() != 1) {
                Log.error("Malformed trained architecture");
            }

            if (trainedArchitecture.getInputs().size() != 2) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + "Number of critic network inputs is wrong. Critic network has two inputs," +
                        "first needs to be a state input and second needs to be the action input.");
            }

            final String stateInput = trainedArchitecture.getInputs().get(0);
            final String actionOutput = trainedArchitecture.getOutputs().get(0);
            final List<Integer> stateDimensions = trainedArchitecture.getDimensions().get(stateInput);
            final List<Integer> actionDimensions = trainedArchitecture.getDimensions().get(actionOutput);
            final Range stateRange = trainedArchitecture.getRanges().get(stateInput);
            final Range actionRange = trainedArchitecture.getRanges().get(actionOutput);
            final String stateType = trainedArchitecture.getTypes().get(stateInput);
            final String actionType = trainedArchitecture.getTypes().get(actionOutput);

            String criticInput1 = criticNetwork.getInputs().get(0);
            String criticInput2 = criticNetwork.getInputs().get(1);

            if (criticNetwork.getDimensions().get(criticInput1).equals(stateDimensions)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                    + " Declared critic network is not a critic: Dimensions of first input of critic architecture must be" +
                        " equal to state's dimensions "
                    + stateDimensions.stream().map(Object::toString).collect(Collectors.joining("{", ",", "}"))
                    + ".", configurationSymbol.getSourcePosition());
            }

            if (criticNetwork.getDimensions().get(criticInput2).equals(actionDimensions)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Dimensions of second input of critic architecture must be" +
                        " equal to action's dimensions "
                        + actionDimensions.stream().map(Object::toString).collect(Collectors.joining("{", ",", "}"))
                        + ".", configurationSymbol.getSourcePosition());
            }

            if (criticNetwork.getRanges().get(criticInput1).equals(stateRange)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Ranges of first input of critic architecture must be" +
                        " equal to state's ranges "
                        + stateRange.toString()
                        + ".", configurationSymbol.getSourcePosition());
            }

            if (criticNetwork.getRanges().get(criticInput2).equals(actionRange)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Ranges of second input of critic architecture must be" +
                        " equal to action's ranges "
                        + actionRange.toString()
                        + ".", configurationSymbol.getSourcePosition());
            }

            if (criticNetwork.getTypes().get(criticInput1).equals(stateType)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Type of first input of critic architecture must be" +
                        " equal to state's types "
                        + stateType
                        + ".", configurationSymbol.getSourcePosition());
            }

            if (criticNetwork.getTypes().get(criticInput2).equals(actionType)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Type of second input of critic architecture must be" +
                        " equal to action's types "
                        + stateType
                        + ".", configurationSymbol.getSourcePosition());
            }
        }
    }
}
