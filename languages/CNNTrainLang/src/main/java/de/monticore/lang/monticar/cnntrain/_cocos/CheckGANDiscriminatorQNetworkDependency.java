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

public class CheckGANDiscriminatorQNetworkDependency implements CNNTrainConfigurationSymbolCoCo {

    public void CheckGANNetworkPorts() { }

    @Override
    public void check(ConfigurationSymbol configurationSymbol) {

        NNArchitectureSymbol dis = configurationSymbol.getDiscriminatorNetwork().get();
        Optional<NNArchitectureSymbol> qnet = configurationSymbol.getQNetwork();

        if(qnet.isPresent() && dis.getOutputs().size() != 2)
            Log.error("0" + ErrorCodes.GAN_ARCHITECTURE_ERROR + " Discriminator needs exactly 2 output " +
                    "ports when q-network is given");

        if(!qnet.isPresent() && dis.getOutputs().size() != 1)
            Log.error("0" + ErrorCodes.GAN_ARCHITECTURE_ERROR + " Discriminator needs exactly 1 output " +
                    "port when no q-network is given");

        if(qnet.isPresent() && dis.getOutputs().size() == 2)
            if(!dis.getOutputs().get(1).equals("features"))
                Log.error("0" + ErrorCodes.GAN_ARCHITECTURE_ERROR + " Second output of discriminator network " +
                        "has to be named features when " +
                        "q-network is given");

        if(qnet.isPresent() && !qnet.get().getInputs().get(0).equals("features"))
                Log.error("0" + ErrorCodes.GAN_ARCHITECTURE_ERROR + " Input to q-network needs to be named features");
    }
}
