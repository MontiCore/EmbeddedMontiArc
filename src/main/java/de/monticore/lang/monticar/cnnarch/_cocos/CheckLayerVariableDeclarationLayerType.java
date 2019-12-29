/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckLayerVariableDeclarationLayerType extends CNNArchSymbolCoCo {

    @Override
    public void check(VariableDeclarationSymbol sym) {
        if (sym instanceof LayerVariableDeclarationSymbol) {
            LayerVariableDeclarationSymbol layerVariableDeclaration = (LayerVariableDeclarationSymbol) sym;

            // Only allow predefined layers to be declared as layer variable
            if (!layerVariableDeclaration.getLayer().getDeclaration().isPredefined()) {
                Log.error("0" + ErrorCodes.ILLEGAL_LAYER_USE + " Illegal layer use. " +
                                "Only predefined layers can be used as a type for a layer variable.",
                          sym.getSourcePosition());
            }
        }
    }
}
