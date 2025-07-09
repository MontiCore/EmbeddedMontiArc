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

public class CheckGANQNetworkhasOneInput implements CNNTrainConfigurationSymbolCoCo {

    public void CheckGANNetworkPorts() { }

    @Override
    public void check(ConfigurationSymbol configurationSymbol) {

        Optional<NNArchitectureSymbol> qnet = configurationSymbol.getQNetwork();

        if(qnet.isPresent() && qnet.get().getInputs().size() != 1)
            Log.error("0" + ErrorCodes.GAN_ARCHITECTURE_ERROR + " Q-Network has more then one input, " +
                    "but is supposed to only have one");
    }
}
