/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerDeclarationSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.PredefinedLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class CheckVariableMember extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureElementSymbol sym) {
        if (sym instanceof VariableSymbol) {
            checkVariable((VariableSymbol) sym);
        }
    }

    public void checkVariable(VariableSymbol variable) {
        if (variable.getType() == VariableSymbol.Type.LAYER) {
            LayerDeclarationSymbol layerDeclaration = variable.getLayerVariableDeclaration().getLayer().getDeclaration();

            if (layerDeclaration.isPredefined() && ((PredefinedLayerDeclaration) layerDeclaration).getArrayLength(variable.getMember()) == 0) {
                Log.error("0" + ErrorCodes.INVALID_MEMBER + " Layer has no member " + variable.getMember().toString().toLowerCase() + ". ",
                          variable.getSourcePosition());
            }

            if (variable.getArrayAccess().isPresent()) {
                Optional<Integer> arrayAccess = variable.getArrayAccess().get().getIntValue();
                int arrayLength = 0;

                if (layerDeclaration.isPredefined()) {
                    arrayLength = ((PredefinedLayerDeclaration) layerDeclaration).getArrayLength(variable.getMember());
                }

                String name = variable.getName() + "." + variable.getMember().toString().toLowerCase();
                if (arrayAccess.isPresent() && arrayLength == 1) {
                    Log.error("0" + ErrorCodes.INVALID_ARRAY_ACCESS + " The layer variable '" + name +
                                    "' does not support array access. "
                            , variable.getSourcePosition());
                } else if (!arrayAccess.isPresent() || arrayAccess.get() < 0 || arrayAccess.get() >= arrayLength) {
                    Log.error("0" + ErrorCodes.INVALID_ARRAY_ACCESS + " The layer variable array access value of '" + name +
                                    "' must be an integer between 0 and " + (arrayLength - 1) + ". " +
                                    "The current value is: " + variable.getArrayAccess().get().getValue().get().toString()
                            , variable.getSourcePosition());
                }

                //
            }
        }

        if (variable.getType() == VariableSymbol.Type.IO && variable.getMember() != VariableSymbol.Member.NONE) {
            Log.error("0" + ErrorCodes.INVALID_MEMBER + " IO variables have no member. ", variable.getSourcePosition());
        }
    }
}
