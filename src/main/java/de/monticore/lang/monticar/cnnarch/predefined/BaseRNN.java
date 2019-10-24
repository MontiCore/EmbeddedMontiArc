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

abstract public class BaseRNN extends PredefinedLayerDeclaration {

    protected int numberOfStates = 1;

    public BaseRNN(String name) {
        super(name);
    }

    @Override
    public boolean isTrainable(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.NONE;
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        boolean bidirectional = layer.getBooleanValue(AllPredefinedLayers.BIDIRECTIONAL_NAME).get();
        int units = layer.getIntValue(AllPredefinedLayers.UNITS_NAME).get();

        if (member == VariableSymbol.Member.STATE) {
            int layers = layer.getIntValue(AllPredefinedLayers.LAYERS_NAME).get();

            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(numberOfStates)
                    .height(bidirectional ? 2 * layers : layers)
                    .width(units)
                    .elementType("-oo", "oo")
                    .build());
        }
        else {
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(layer.getInputTypes().get(0).getChannels())
                    .height(bidirectional ? 2 * units : units)
                    .elementType("-oo", "oo")
                    .build());
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        boolean bidirectional = layer.getBooleanValue(AllPredefinedLayers.BIDIRECTIONAL_NAME).get();
        int units = layer.getIntValue(AllPredefinedLayers.UNITS_NAME).get();
        int layers = layer.getIntValue(AllPredefinedLayers.LAYERS_NAME).get();

        if (member == VariableSymbol.Member.STATE) {
            errorIfInputSizeIsNotOne(inputTypes, layer);
            errorIfInputChannelSizeIsInvalid(inputTypes, layer, numberOfStates);
            errorIfInputHeightIsInvalid(inputTypes, layer, bidirectional ? 2 * layers : layers);
            errorIfInputWidthIsInvalid(inputTypes, layer, units);
        }
        else {
            errorIfInputSizeIsNotOne(inputTypes, layer);
            errorIfInputChannelSizeIsInvalid(inputTypes, layer, layer.getInputTypes().get(0).getChannels());
            errorIfInputWidthIsInvalid(inputTypes, layer, 1);
        }
    }

    @Override
    public boolean isValidMember(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.NONE ||
               member == VariableSymbol.Member.OUTPUT ||
               member == VariableSymbol.Member.STATE;
    }

    @Override
    public boolean canBeInput(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.OUTPUT || member == VariableSymbol.Member.STATE;
    }

    @Override
    public boolean canBeOutput(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.NONE || member == VariableSymbol.Member.STATE;
    }

    protected static List<ParameterSymbol> createParameters() {
        return new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.UNITS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.LAYERS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.BIDIRECTIONAL_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(false)
                        .build()));
    }
}
