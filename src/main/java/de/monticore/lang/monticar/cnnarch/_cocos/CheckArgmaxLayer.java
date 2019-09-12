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
