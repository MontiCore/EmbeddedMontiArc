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

public class Memory extends PredefinedLayerDeclaration {

    private Memory() {
        super(AllPredefinedLayers.MEMORY_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        int querySize = layer.getIntValue(AllPredefinedLayers.QUERY_SIZE_NAME).get();

        return Collections.singletonList(new ArchTypeSymbol.Builder()
            .channels(1)
            .height(querySize)
            .width(1)
            .elementType("-oo", "oo")
            .build());
    }
    
    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Memory create(){
        Memory declaration = new Memory();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SUB_KEY_SIZE_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERY_SIZE_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(512)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.ACT_QUERY_NAME)
                        .constraints(Constraints.ACTIVATION_TYPE)
                        .defaultValue("linear")
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.K_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NUM_HEADS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
