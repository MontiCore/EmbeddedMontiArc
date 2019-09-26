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
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ExpandDims extends PredefinedLayerDeclaration {

    private ExpandDims() {
        super(AllPredefinedLayers.EXPAND_DIMS_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        int dim = layer.getIntValue(AllPredefinedLayers.DIM_NAME).get();
        int channels = layer.getInputTypes().get(0).getChannels();
        int height = layer.getInputTypes().get(0).getHeight();
        int width = layer.getInputTypes().get(0).getWidth();

        if (dim == 0) {
            width = height;
            height = channels;
            channels = 1;
        }else if (dim == 1) {
            width = height;
            height = 1;
        }

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(channels)
                .height(height)
                .width(width)
                .elementType("-oo", "oo")
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
        errorIfDimNotFeasible(inputTypes, layer);
    }

    public static ExpandDims create(){
        ExpandDims declaration = new ExpandDims();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.NON_NEGATIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
