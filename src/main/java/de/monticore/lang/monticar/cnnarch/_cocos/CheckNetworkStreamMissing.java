/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckNetworkStreamMissing extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureSymbol architecture) {
        boolean hasTrainableStream = false;

        for (CompositeElementSymbol stream : architecture.getStreams()) {
            hasTrainableStream |= stream.isTrainable();
        }

        if (!hasTrainableStream) {
            Log.error("0" + ErrorCodes.MISSING_TRAINABLE_STREAM + " The architecture has no trainable stream. "
                    , architecture.getSourcePosition());
        }
    }

}
