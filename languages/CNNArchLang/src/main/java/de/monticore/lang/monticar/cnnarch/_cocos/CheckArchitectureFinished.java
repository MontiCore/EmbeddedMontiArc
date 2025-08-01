/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.NetworkInstructionSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Log;

public class CheckArchitectureFinished extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureSymbol architecture) {
        for (NetworkInstructionSymbol networkInstruction : architecture.getNetworkInstructions()) {
            if (!networkInstruction.getBody().getOutputTypes().isEmpty()) {
                Log.error("0" + ErrorCodes.UNFINISHED_ARCHITECTURE + " The architecture is not finished. " +
                                "There are still open streams at the end of the architecture. "
                        , networkInstruction.getSourcePosition());
            }
        }

        if (architecture.getInputs().isEmpty()){
            Log.error("0" + ErrorCodes.UNFINISHED_ARCHITECTURE + " The architecture has no inputs. "
                    , architecture.getSourcePosition());
        }
        if (architecture.getOutputs().isEmpty()){
            Log.error("0" + ErrorCodes.UNFINISHED_ARCHITECTURE + " The architecture has no outputs. "
                    , architecture.getSourcePosition());
        }
    }

}
