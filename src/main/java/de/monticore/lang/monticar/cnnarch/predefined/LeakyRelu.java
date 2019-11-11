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
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class LeakyRelu extends PredefinedLayerDeclaration {

    private LeakyRelu() { super(AllPredefinedLayers.LEAKY_RELU_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(layer.getInputTypes().get(0).getChannels())
                        .height(layer.getInputTypes().get(0).getHeight())
                        .width(layer.getInputTypes().get(0).getWidth())
                        .elementType("0", "oo")
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static LeakyRelu create(){
        LeakyRelu declaration = new LeakyRelu();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.ALPHA_NAME)
                        .constraints(Constraints.POSITIVE)
                        .defaultValue(0)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
