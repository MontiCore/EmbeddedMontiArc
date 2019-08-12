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

import java.util.*;

public class RNN extends PredefinedLayerDeclaration {

    private RNN() {
        super(AllPredefinedLayers.RNN_NAME);
    }

    @Override
    public boolean isTrainable(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.NONE;
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int units = layer.getIntValue(AllPredefinedLayers.UNITS_NAME).get();

        if (member == VariableSymbol.Member.STATE) {
            int layers = layer.getIntValue(AllPredefinedLayers.LAYERS_NAME).get();

            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(layers)
                    .height(units)
                    .elementType("-oo", "oo")
                    .build());
        }
        else {
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(layer.getInputTypes().get(0).getChannels())
                    .height(units)
                    .elementType("-oo", "oo")
                    .build());
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int units = layer.getIntValue(AllPredefinedLayers.UNITS_NAME).get();
        int layers = layer.getIntValue(AllPredefinedLayers.LAYERS_NAME).get();

        if (member == VariableSymbol.Member.STATE) {
            errorIfInputSizeIsNotOne(inputTypes, layer);
            errorIfInputChannelSizeIsInvalid(inputTypes, layer, layers);
            errorIfInputHeightIsInvalid(inputTypes, layer, units);
            errorIfInputWidthIsInvalid(inputTypes, layer, 1);
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

    public static RNN create() {
        RNN declaration = new RNN();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.UNITS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.LAYERS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
