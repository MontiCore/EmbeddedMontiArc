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

public class Squeeze extends PredefinedLayerDeclaration {

    private Squeeze() {
        super(AllPredefinedLayers.SQUEEZE_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        List<Integer> dimensions = layer.getInputTypes().get(0).getDimensions();

        int axis = layer.getIntValue(AllPredefinedLayers.AXIS_NAME).get();

        if (axis == -1) {
            dimensions.remove(new Integer(1));
        } else {
            dimensions.remove(axis);
        }

        while (dimensions.size() < 3) {
            dimensions.add(1);
        }

        int channels = dimensions.get(0);
        int height = dimensions.get(1);
        int width = dimensions.get(2);

        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(channels)
                        .height(height)
                        .width(width)
                        .elementType(layer.getInputTypes().get(0).getDomain())
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);

        int axis = layer.getIntValue(AllPredefinedLayers.AXIS_NAME).get();

        if (axis == 0) {
            errorIfInputChannelSizeIsInvalid(inputTypes, layer, 1);
        } else if (axis == 1) {
            errorIfInputHeightIsInvalid(inputTypes, layer, 1);
        } else if (axis == 2) {
            errorIfInputWidthIsInvalid(inputTypes, layer, 1);
        }
    }

    public static Squeeze create(){
        Squeeze declaration = new Squeeze();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.AXIS_NAME)
                        .constraints(Constraints.NULLABLE_AXIS)
                        .defaultValue(-1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
