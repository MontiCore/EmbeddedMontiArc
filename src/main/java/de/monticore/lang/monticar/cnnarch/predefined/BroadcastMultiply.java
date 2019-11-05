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
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BroadcastMultiply extends PredefinedLayerDeclaration {

    private BroadcastMultiply() {
        super(AllPredefinedLayers.BROADCAST_MULTIPLY_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        List<String> range = computeStartAndEndValue(layer.getInputTypes(), Rational::times, Rational::times);

        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(Math.max(layer.getInputTypes().get(0).getChannels(), layer.getInputTypes().get(1).getChannels()))
                        .height(Math.max(layer.getInputTypes().get(0).getHeight(), layer.getInputTypes().get(1).getHeight()))
                        .width(Math.max(layer.getInputTypes().get(0).getWidth(), layer.getInputTypes().get(1).getWidth()))
                        .elementType(range.get(0), range.get(1))
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
        if (inputTypes.size() == 1){
            Log.warn("BroadcastMultiply layer has only one input stream. Layer can be removed." , layer.getSourcePosition());
        }
    }

    public static BroadcastMultiply create(){
        BroadcastMultiply declaration = new BroadcastMultiply();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
