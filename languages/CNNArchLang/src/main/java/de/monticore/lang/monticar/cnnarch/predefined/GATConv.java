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

public class GATConv extends PredefinedLayerDeclaration {

    private GATConv() {
        super(AllPredefinedLayers.GAT_CONV_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int height = layer.getIntValue(AllPredefinedLayers.NODES_NAME).get();
        int width = layer.getIntValue(AllPredefinedLayers.NUM_HEADS_NAME).get() * layer.getIntValue(AllPredefinedLayers.OUTPUT_DIM_NAME).get();

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(1)
                .height(height)
                .width(width)
                .elementType(String.valueOf(0), "oo")
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
    }

    public static GATConv create(){
        GATConv declaration = new GATConv();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.INPUT_DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.OUTPUT_DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NUM_HEADS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NODES_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
