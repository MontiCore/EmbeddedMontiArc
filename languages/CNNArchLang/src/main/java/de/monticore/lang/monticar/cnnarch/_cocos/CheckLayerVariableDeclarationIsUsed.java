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

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class CheckLayerVariableDeclarationIsUsed extends CNNArchSymbolCoCo {

    @Override
    public void check(VariableDeclarationSymbol sym) {
        if (sym instanceof LayerVariableDeclarationSymbol) {
            LayerVariableDeclarationSymbol layerVariableDeclaration = (LayerVariableDeclarationSymbol) sym;

            boolean isUsed = false;

            Set<String> allowedUnusedLayers = new HashSet();
            allowedUnusedLayers.add("attention");

            for (NetworkInstructionSymbol networkInstruction : layerVariableDeclaration.getLayer().getArchitecture().getNetworkInstructions()) {
                Collection<ArchitectureElementSymbol> elements
                        = networkInstruction.getBody().getSpannedScope().resolveMany(layerVariableDeclaration.getName(), ArchitectureElementSymbol.KIND);

                for (ArchitectureElementSymbol element : elements) {
                    if (element instanceof VariableSymbol && ((VariableSymbol) element).getMember() == VariableSymbol.Member.NONE) {
                        isUsed = true;
                        break;
                    }
                }

                if (isUsed) {
                    break;
                }
            }

            if(allowedUnusedLayers.contains(sym.getName())){
                isUsed = true;
            }

            if (!isUsed) {
                Log.error("0" + ErrorCodes.UNUSED_LAYER + " Unused layer. " +
                                "Declared layer variables need to be used as layer at least once.",
                        sym.getSourcePosition());
            }
        }
    }
}
