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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Concatenate extends PredefinedLayerDeclaration {

    private Concatenate() {
        super(AllPredefinedLayers.CONCATENATE_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int height = 0;
        int width = 0;
        int channels = 0;

        int dim = layer.getIntValue(AllPredefinedLayers.DIM_NAME).get();

        List<String> range = computeStartAndEndValue(layer.getInputTypes(), (x,y) -> x.isLessThan(y) ? x : y, (x,y) -> x.isLessThan(y) ? y : x);

        if(dim==0){
            for (ArchTypeSymbol inputShape : layer.getInputTypes()) {
                channels += inputShape.getChannels();
            }
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(channels)
                    .height(layer.getInputTypes().get(0).getHeight())
                    .width(layer.getInputTypes().get(0).getWidth())
                    .elementType(range.get(0), range.get(1))
                    .build());
        }else if(dim==1){
            for (ArchTypeSymbol inputShape : layer.getInputTypes()) {
                height += inputShape.getHeight();
            }
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(layer.getInputTypes().get(0).getChannels())
                    .height(height)
                    .width(layer.getInputTypes().get(0).getWidth())
                    .elementType(range.get(0), range.get(1))
                    .build());
        } else if(dim==2){
            for (ArchTypeSymbol inputShape : layer.getInputTypes()) {
                width += inputShape.getWidth();
            }
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(layer.getInputTypes().get(0).getChannels())
                    .height(layer.getInputTypes().get(0).getHeight())
                    .width(width)
                    .elementType(range.get(0), range.get(1))
                    .build());
        }else{
            return new ArrayList<>();
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        if (!inputTypes.isEmpty()) {
            List<Integer> channelList = new ArrayList<>();
            List<Integer> heightList = new ArrayList<>();
            List<Integer> widthList = new ArrayList<>();
            for (ArchTypeSymbol shape : inputTypes){
                heightList.add(shape.getHeight());
                widthList.add(shape.getWidth());
                channelList.add(shape.getChannels());
            }
            int countEqualcHannels = (int)channelList.stream().distinct().count();
            int countEqualHeights = (int)heightList.stream().distinct().count();
            int countEqualWidths = (int)widthList.stream().distinct().count();
            if (countEqualHeights != 1 && countEqualWidths != 1 && countEqualcHannels != 1){
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                                "Concatenation of inputs with different resolutions is not possible. " +
                                "Input channels: " + Joiners.COMMA.join(channelList) + ". " +
                                "Input heights: " + Joiners.COMMA.join(heightList) + ". " +
                                "Input widths: " + Joiners.COMMA.join(widthList) + ". "
                        , layer.getSourcePosition());
            }
        }
        else {
            errorIfInputIsEmpty(inputTypes, layer);
        }
    }

    public static Concatenate create(){
        Concatenate declaration = new Concatenate();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.NON_NEGATIVE)
                        .defaultValue(1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}