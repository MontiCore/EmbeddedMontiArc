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

public class OneHot extends PredefinedLayerDeclaration {

    private OneHot() {
        super(AllPredefinedLayers.ONE_HOT_NAME);
    }

    @Override
    public boolean isTrainable() {
        return false;
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        // TODO: Execute this code somewhere before checkInput(), for now size parameter is required
        /*if(layer.getOutputElement().get() instanceof IOSymbol && layer.getOutputElement().get().isOutput()) {
            int outputChannels = ((IOSymbol) layer.getOutputElement().get()).getDefinition().getType().getChannels();

            layer.setIntValue(AllPredefinedLayers.SIZE_NAME, outputChannels);
        }*/

        int size = layer.getIntValue(AllPredefinedLayers.SIZE_NAME).get();

        /*if (size == 0) {
            Log.error("0" + ErrorCodes.MISSING_ARGUMENT + " Missing argument. The argument 'size' is in this case required. "
                      , layer.getSourcePosition());

        }*/

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(size)
                .height(1)
                .width(1)
                .elementType("0", "1")
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
        errorIfInputChannelSizeIsInvalid(inputTypes, layer, 1);
        errorIfInputHeightIsInvalid(inputTypes, layer, 1);
        errorIfInputWidthIsInvalid(inputTypes, layer, 1);

        // Check range of input
        ASTElementType domain = inputTypes.get(0).getDomain();

        if (!domain.isNaturalNumber() && !domain.isWholeNumber()) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Input needs to be natural or whole. "
                      , layer.getSourcePosition());
        }
        else {
            if (!domain.getRangeOpt().isPresent()) {
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Range is missing. "
                          , layer.getSourcePosition());
            }
            else {
                ASTRange range = domain.getRangeOpt().get();

                if (!range.getMin().getNumber().isPresent() || !range.getMax().getNumber().isPresent()) {
                    Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Minimum or maximum is missing. "
                              , layer.getSourcePosition());
                }
                else {
                    double min = range.getMin().getNumber().get();
                    double max = range.getMax().getNumber().get();

                    // Check if minimum >= 0
                    if (min < 0) {
                        Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Minimum needs to be bigger than 0. "
                                  , layer.getSourcePosition());
                    }

                    int size = layer.getIntValue(AllPredefinedLayers.SIZE_NAME).get();

                    // Check if maximum < size
                    if (max >= size) {
                        Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: "
                                  + "Maximum needs to be smaller than size " + size + ". "
                                  , layer.getSourcePosition());
                    }
                }
            }
        }

    }

    public static OneHot create(){
        OneHot declaration = new OneHot();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SIZE_NAME)
                        .constraints(Constraints.POSITIVE, Constraints.INTEGER)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
