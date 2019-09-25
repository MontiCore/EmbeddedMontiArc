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

import java.util.Collection;

public class CheckLayerVariableDeclarationIsUsed extends CNNArchSymbolCoCo {

    @Override
    public void check(VariableDeclarationSymbol sym) {
        if (sym instanceof LayerVariableDeclarationSymbol) {
            LayerVariableDeclarationSymbol layerVariableDeclaration = (LayerVariableDeclarationSymbol) sym;

            boolean isUsed = false;

            for (SerialCompositeElementSymbol stream : layerVariableDeclaration.getLayer().getArchitecture().getStreams()) {
                Collection<ArchitectureElementSymbol> elements =
                        stream.getSpannedScope().resolveMany(layerVariableDeclaration.getName(), ArchitectureElementSymbol.KIND);

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

            if (!isUsed) {
                Log.error("0" + ErrorCodes.UNUSED_LAYER + " Unused layer. " +
                                "Declared layer variables need to be used as layer at least once.",
                        sym.getSourcePosition());
            }
        }
    }
}
