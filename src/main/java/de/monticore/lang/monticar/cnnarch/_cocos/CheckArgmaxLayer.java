/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.se_rwth.commons.logging.Log;

public class CheckArgmaxLayer extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureElementSymbol symbol) {
        if (symbol instanceof LayerSymbol && symbol.getName().equals(AllPredefinedLayers.ARG_MAX_NAME)) {
            checkArgmaxBeforeOutput((LayerSymbol) symbol);
        }
    }

    public void checkArgmaxBeforeOutput(LayerSymbol layer) {
        if(!(layer.getOutputElement().get() instanceof VariableSymbol)){
            Log.error("0" + ErrorCodes.ILLEGAL_LAYER_USE + " ArgMax Layer must be applied directly before an output symbol.");
        }
    }
}
