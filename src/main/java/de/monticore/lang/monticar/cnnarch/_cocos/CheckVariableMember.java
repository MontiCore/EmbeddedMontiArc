/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerDeclarationSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.PredefinedLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

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

            if (layerDeclaration.isPredefined() && !((PredefinedLayerDeclaration) layerDeclaration).isValidMember(variable.getMember())) {
                Log.error("0" + ErrorCodes.INVALID_MEMBER + " Layer has no member " + variable.getMember().toString().toLowerCase() + ". ",
                          variable.getSourcePosition());
            }

            if (variable.getArrayAccess().isPresent()) {
                Log.error("0" + ErrorCodes.INVALID_MEMBER + " Currently layer variable array access is not implemented. ",
                          variable.getSourcePosition());
            }
        }

        if (variable.getType() == VariableSymbol.Type.IO && variable.getMember() != VariableSymbol.Member.NONE) {
            Log.error("0" + ErrorCodes.INVALID_MEMBER + " IO variables have no member. ", variable.getSourcePosition());
        }
    }
}
