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

import java.util.Optional;

public class CheckGANGeneratorQNetworkDependency implements CNNTrainConfigurationSymbolCoCo {

    public void CheckGANNetworkPorts() { }

    @Override
    public void check(ConfigurationSymbol configurationSymbol) {

        NNArchitectureSymbol gen = configurationSymbol.getTrainedArchitecture().get();
        Optional<NNArchitectureSymbol> qnet = configurationSymbol.getQNetwork();

        if(qnet.isPresent())
            if(!gen.getInputs().containsAll(qnet.get().getOutputs()))
                Log.error("0" + ErrorCodes.GAN_ARCHITECTURE_ERROR + " Generator input does not contain all " +
                    "latent-codes outputted by q-network");
    }
}
