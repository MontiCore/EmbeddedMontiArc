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
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 *
 */
public class CheckCriticNetworkHasExactlyAOneDimensionalOutput implements CNNTrainConfigurationSymbolCoCo {

    @Override
    public void check(ConfigurationSymbol configurationSymbol) {
        if (configurationSymbol.getCriticNetwork().isPresent()) {
            NNArchitectureSymbol criticNetwork = configurationSymbol.getCriticNetwork().get();

            if (criticNetwork.getOutputs().size() > 1) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR
                        + " The critic network has more than one outputs", criticNetwork.getSourcePosition());
            }
            final String outputName = criticNetwork.getOutputs().get(0);
            List<Integer> dimensions = criticNetwork.getDimensions().get(outputName);

            if (dimensions.size() != 1 || dimensions.get(0) != 1) {
                Log.error("0" + ErrorCodes.CRITIC_NETWORK_ERROR + " The output " + outputName
                        + " of critic network is not a one-dimensional vector", configurationSymbol.getSourcePosition());
            }
        }
    }
}
