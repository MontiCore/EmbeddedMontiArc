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
public class CheckAdaNetTooManyLayers extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureSymbol architecture) {
        for (NetworkInstructionSymbol networkInstruction : architecture.getNetworkInstructions()) {
            int counter = 0;
            for(ArchitectureElementSymbol layer:networkInstruction.getBody().getElements()){
                counter += layer.getName().equals(AllPredefinedLayers.AdaNet_Name)?1:0;
            }
            if(counter >1){
                String msg = "0" + ErrorCodes.ADANET_TOO_MANY_ADANET_LAYER + String.format(" an architecture using AdaNet must not contain more than 1 AdaNet Layer, found %d",counter);
                Log.error(msg);
            }
        }



    }
}