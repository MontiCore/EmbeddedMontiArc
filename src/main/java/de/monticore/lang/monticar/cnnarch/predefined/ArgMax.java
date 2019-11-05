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
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ArgMax extends PredefinedLayerDeclaration {

    private ArgMax() {
        super(AllPredefinedLayers.ARG_MAX_NAME);
    }

    @Override
    public boolean isTrainable() {
        return false;
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        ArchTypeSymbol inputType = layer.getInputTypes().get(0);

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(1)
                .height(1)
                .width(1)
                .elementType("Z", "0", String.valueOf(inputType.getChannels() - 1))
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);

        errorIfInputHeightIsInvalid(inputTypes, layer, 1);
        errorIfInputWidthIsInvalid(inputTypes, layer, 1);
    }

    public static ArgMax create(){
        ArgMax declaration = new ArgMax();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
