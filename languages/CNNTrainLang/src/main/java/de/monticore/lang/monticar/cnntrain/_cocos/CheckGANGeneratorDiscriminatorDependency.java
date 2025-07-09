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

public class CheckGANGeneratorDiscriminatorDependency implements CNNTrainConfigurationSymbolCoCo {

    public void CheckGANNetworkPorts() { }

    @Override
    public void check(ConfigurationSymbol configurationSymbol) {

        NNArchitectureSymbol gen = configurationSymbol.getTrainedArchitecture().get();
        NNArchitectureSymbol dis = configurationSymbol.getDiscriminatorNetwork().get();

        if(!gen.getOutputs().get(0).equals(dis.getInputs().get(0)))
            Log.error("0" + ErrorCodes.GAN_ARCHITECTURE_ERROR + " The generator networks output name does not " +
                    "fit the first discriminators input name");
    }
}
