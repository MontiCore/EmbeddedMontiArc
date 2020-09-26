/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
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

            if (criticNetwork.getInputs().size() != 2) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + "Number of critic network inputs is wrong. Critic network has two inputs," +
                        "first needs to be a state input and second needs to be the action input.");
            }

            if (!criticNetwork.getDimensions().get(criticInput1).equals(stateDimensions)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                    + " Declared critic network is not a critic: Dimensions of first input of critic architecture must be" +
                        " equal to state's dimensions "
                    + stateDimensions.stream().map(Object::toString).collect(Collectors.joining(",", "{", "}"))
                    + ".", configurationSymbol.getSourcePosition());
            }

            if (!criticNetwork.getDimensions().get(criticInput2).equals(actionDimensions)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Dimensions of second input of critic architecture must be" +
                        " equal to action's dimensions "
                        + actionDimensions.stream().map(Object::toString).collect(Collectors.joining(",", "{", "}"))
                        + ".", configurationSymbol.getSourcePosition());
            }

            if (!criticNetwork.getRanges().get(criticInput1).equals(stateRange)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Ranges of first input of critic architecture must be" +
                        " equal to state's ranges "
                        + stateRange.toString()
                        + ".", configurationSymbol.getSourcePosition());
            }

            if (!criticNetwork.getRanges().get(criticInput2).equals(actionRange)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Ranges of second input of critic architecture must be" +
                        " equal to action's ranges "
                        + actionRange.toString()
                        + ".", configurationSymbol.getSourcePosition());
            }

            if (!criticNetwork.getTypes().get(criticInput1).equals(stateType)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Type of first input of critic architecture must be" +
                        " equal to state's types "
                        + stateType
                        + ".", configurationSymbol.getSourcePosition());
            }

            if (!criticNetwork.getTypes().get(criticInput2).equals(actionType)) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " Declared critic network is not a critic: Type of second input of critic architecture must be" +
                        " equal to action's types "
                        + stateType
                        + ".", configurationSymbol.getSourcePosition());
            }
        }
    }
}
