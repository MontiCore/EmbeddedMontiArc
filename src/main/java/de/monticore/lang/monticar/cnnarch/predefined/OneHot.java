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
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class OneHot extends PredefinedLayerDeclaration {

    private static int channels;

    private OneHot() {
        super(AllPredefinedLayers.ONE_HOT_NAME);
    }


    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer) {

        channels = inputTypes.get(0).getChannels();

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(layer.getIntValue(AllPredefinedLayers.ONE_HOT_SIZE_NAME).get())
                .height(1)
                .width(1)
                .elementType("0", "1")
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
        //errorIfInputSizeUnequalToOnehotSize(inputTypes, layer);
    }

    public static OneHot create(){
        OneHot declaration = new OneHot();
        List<VariableSymbol> parameters = new ArrayList<>(Arrays.asList(
                new VariableSymbol.Builder()
                        .name(AllPredefinedLayers.ONE_HOT_SIZE_NAME)
                        .constraints(Constraints.POSITIVE, Constraints.INTEGER)
                        .defaultValue(channels)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
