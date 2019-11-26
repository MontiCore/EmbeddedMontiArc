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

public class Reshape extends PredefinedLayerDeclaration {

    private Reshape() {
        super(AllPredefinedLayers.RESHAPE_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        List<Integer> shape = layer.getIntTupleValue(AllPredefinedLayers.SHAPE_NAME).get();

        int channels = -1;
        int height = -1;
        int width = -1;


        if (shape.size() >= 3) {
            width = shape.get(2);
        }

        if (shape.size() >= 2) {
            height = shape.get(1);
        }

        if (shape.size() >= 1) {
            channels = shape.get(0);
        } else {
            Log.error("0" + ErrorCodes.ILLEGAL_PARAMETER_VALUE + "\"Shape\" argument needs to contain at least one entry"
                    , layer.getSourcePosition());
        }

        int totalSize = layer.getInputTypes().get(0).getChannels() * layer.getInputTypes().get(0).getHeight() * layer.getInputTypes().get(0).getWidth();
        int newTotalSize = shape.stream().reduce(1, (x, y) -> x * y);

        if (totalSize != newTotalSize && newTotalSize != 0) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + "The input of Reshape layer cannot be reshaped to the given shape. "
                    + "Source and target shape have a different amount of total values", layer.getSourcePosition());
        }

        if (newTotalSize != 0) {

            if (width != -1) {
                return Collections.singletonList(
                        new ArchTypeSymbol.Builder()
                                .channels(channels)
                                .height(height)
                                .width(width)
                                .elementType(layer.getInputTypes().get(0).getDomain())
                                .build());
            } else if (height != -1) {
                return Collections.singletonList(
                        new ArchTypeSymbol.Builder()
                                .channels(channels)
                                .height(height)
                                .elementType(layer.getInputTypes().get(0).getDomain())
                                .build());
            } else{
                return Collections.singletonList(
                        new ArchTypeSymbol.Builder()
                                .channels(channels)
                                .elementType(layer.getInputTypes().get(0).getDomain())
                                .build());
            }
        }
        return Collections.emptyList();
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Reshape create(){
        Reshape declaration = new Reshape();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SHAPE_NAME)
                        .constraints(Constraints.INTEGER_TUPLE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}