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

public class GraphAvgPool extends PredefinedLayerDeclaration {

    private GraphAvgPool() {
        super(AllPredefinedLayers.GRAPH_AVG_POOL_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(layer.getInputTypes().get(0).getChannels())
                .height(1)
                .width(1)
                .elementType(String.valueOf(0), "oo")
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
    }

    public static GraphAvgPool create(){
        GraphAvgPool declaration = new GraphAvgPool();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
