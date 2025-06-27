/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._cocos;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.NetworkInstructionSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.se_rwth.commons.logging.Log;

public class CheckAdaNetMalFormedArchitecture extends CNNArchSymbolCoCo {
    @Override
    public void check(ArchitectureSymbol architecture) {
        ArchitectureElementSymbol input = architecture.getInputs().get(0);
        ArchitectureElementSymbol output = architecture.getOutputs().get(0);
        boolean toMany = false;
        if (architecture.containsAdaNet()) {
            // if no AdaNet Layer is present skip this test
            for (NetworkInstructionSymbol networkInstruction : architecture.getNetworkInstructions()) {
                for (ArchitectureElementSymbol layer : networkInstruction.getBody().getElements()) {
                    if (!(layer.equals(input) || layer.equals(output))) {
                        // if a non AdaNet Layer is found which is not an input or output log error
                        toMany = toMany | !layer.getName().equals(AllPredefinedLayers.AdaNet_Name);
                    }
                }
            }
            if (toMany) {
                String msg = "0" + ErrorCodes.ADANET_ILLEGAL_ARCH + " an architecture using an AdaNet Layer must pass all other layers within a def block to the adaNet layer";
                Log.error(msg);
            }
        }


    }
}