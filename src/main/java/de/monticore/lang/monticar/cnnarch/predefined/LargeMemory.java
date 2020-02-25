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

public class LargeMemory extends PredefinedLayerDeclaration {

    private LargeMemory() {
        super(AllPredefinedLayers.LARGE_MEMORY_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        int querrySize = layer.getIntValue(AllPredefinedLayers.QUERRY_SIZE_NAME).get();

        return Collections.singletonList(new ArchTypeSymbol.Builder()
            .channels(1)
            .height(querrySize)
            .width(1)
            .elementType("-oo", "oo")
            .build());
    }
    
    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static LargeMemory create(){
        LargeMemory declaration = new LargeMemory();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NUM_SUB_KEYS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERRY_SIZE_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(512)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.K_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
