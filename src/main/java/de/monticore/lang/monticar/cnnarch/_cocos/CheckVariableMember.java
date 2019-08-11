/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
