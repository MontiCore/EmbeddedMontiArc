/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
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
            if (inputTypes.get(0).getDepthIndex() == -1){
                return Collections.singletonList(new ArchTypeSymbol.Builder()
                        .channels(units)
                        .height(1)
                        .width(1)
                        .elementType("-oo", "oo")
                        .build());
            } else {
                
                return Collections.singletonList(
                    new ArchTypeSymbol.Builder()
                        .channels(units)
                        .height(1)
                        .width(1)
                        .depth(1)
                        .elementType("0", "oo")
                        .build());
            }
        }
        else {
            ArchTypeSymbol inputType = layer.getInputTypes().get(0);
            if (inputTypes.get(0).getDepthIndex() == -1){ //2D Cases
                if (inputType.getWidth() == 1) {
                    if (inputType.getHeight() == 1) {
                        return Collections.singletonList(new ArchTypeSymbol.Builder()
                                .channels(units)
                                .height(1)
                                .width(1)
                                .elementType("-oo", "oo")
                                .build());
                    }

                    return Collections.singletonList(new ArchTypeSymbol.Builder()
                            .channels(inputType.getChannels())
                            .height(units)
                            .width(1)
                            .elementType("-oo", "oo")
                            .build());
                }

                return Collections.singletonList(new ArchTypeSymbol.Builder()
                        .channels(inputType.getChannels())
                        .height(inputType.getHeight())
                        .width(units)
                        .elementType("-oo", "oo")
                        .build());
            }   else { //3D Cases
                    if (inputType.getWidth() == 1) {
                        if (inputType.getHeight() == 1) {
                            if (inputType.getDepth() == 1){
                                    return Collections.singletonList(new ArchTypeSymbol.Builder()
                                        .channels(units)
                                        .height(1)
                                        .width(1)
                                        .depth(1)
                                        .elementType("-oo", "oo")
                                        .build());
                            }   return Collections.singletonList(new ArchTypeSymbol.Builder()
                                    .channels(inputType.getChannels())
                                    .height(1)
                                    .width(1)
                                    .depth(units)
                                    .elementType("-oo", "oo")
                                    .build());
                        }   return Collections.singletonList(new ArchTypeSymbol.Builder()
                                    .channels(inputType.getChannels())
                                    .height(units)
                                    .width(1)
                                    .depth(inputType.getDepth())
                                    .elementType("-oo", "oo")
                                    .build());
                    }   return Collections.singletonList(new ArchTypeSymbol.Builder()
                            .channels(inputType.getChannels())
                            .height(inputType.getHeight())
                            .width(units)
                            .depth(inputType.getDepth())
                            .elementType("-oo", "oo")
                            .build());
                    }
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
