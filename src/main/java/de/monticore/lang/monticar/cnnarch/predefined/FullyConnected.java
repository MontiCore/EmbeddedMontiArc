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

public class FullyConnected extends PredefinedLayerDeclaration {

    private FullyConnected() {
        super(AllPredefinedLayers.FULLY_CONNECTED_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        boolean flatten = layer.getBooleanValue(AllPredefinedLayers.FLATTEN_PARAMETER_NAME).get();
        int units = layer.getIntValue(AllPredefinedLayers.UNITS_NAME).get();

        if (flatten) {
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(units)
                    .height(1)
                    .width(1)
                    .elementType("-oo", "oo")
                    .build());
        }
        else {
            ArchTypeSymbol inputType = layer.getInputTypes().get(0);

            if (inputType.getWidth() == 1) {
                if (inputType.getHeight() == 1) {
                    return Collections.singletonList(new ArchTypeSymbol.Builder()
                            .channels(units)
                            .height(1)
                            .width(1)
                            .elementType("-oo", "oo")
                            .build());
                }

                // if input is an RNN state or output. Can be used to store states in layer variables
                if(layer.getInputElement().get() instanceof VariableSymbol && (((VariableSymbol)(layer.getInputElement().get())).getMember() == VariableSymbol.Member.STATE || ((VariableSymbol)((ArchitectureElementSymbol)layer.getInputElement().get())).getMember() == VariableSymbol.Member.OUTPUT)){
                    return Collections.singletonList(new ArchTypeSymbol.Builder()
                            .channels(inputType.getChannels())
                            .height(units)
                            .elementType("-oo", "oo")
                            .build());
                }else {
                    return Collections.singletonList(new ArchTypeSymbol.Builder()
                            .channels(inputType.getChannels())
                            .height(units)
                            .width(1)
                            .elementType("-oo", "oo")
                            .build());
                }
            }

            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(inputType.getChannels())
                    .height(inputType.getHeight())
                    .width(units)
                    .elementType("-oo", "oo")
                    .build());
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static FullyConnected create(){
        FullyConnected declaration = new FullyConnected();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.UNITS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NOBIAS_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(false)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.FLATTEN_PARAMETER_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(true)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
