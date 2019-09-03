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

            for (UnrollSymbol unroll : layerVariableDeclaration.getLayer().getArchitecture().getUnrolls()) {
               for (ArchitectureElementSymbol element : unroll.getBodiesForAllTimesteps().get(0).getElements()) {
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
