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
import java.util.Optional;

public class ReplayMemory extends PredefinedLayerDeclaration {

    private ReplayMemory() {
        super(AllPredefinedLayers.REPLAY_MEMORY_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        return Collections.singletonList(new ArchTypeSymbol.Builder()
            .channels(1)
            .height(1)
            .width(1)
            .elementType("-oo", "oo")
            .build());
        
    }
    
    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static ReplayMemory create(){
        ReplayMemory declaration = new ReplayMemory();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.USE_REPLAY_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(true)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.REPLAY_INTERVAL_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.REPLAY_BATCH_SIZE_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE_OR_MINUS_ONE)
                        .defaultValue(-1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.REPLAY_STEPS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.REPLAY_GRADIENT_STEPS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.REPLAY_MEMORY_STORE_PROB_NAME)
                        .constraints(Constraints.NUMBER, Constraints.BETWEEN_ZERO_AND_ONE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.USE_LOCAL_ADAPTION_NAME)
                        .constraints(Constraints.BOOLEAN, Constraints.POSITIVE)
                        .defaultValue(true)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.LOCAL_ADAPTION_GRADIENT_STEPS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.REPLAY_MEMORY_STORE_DIST_MEASURE_NAME)
                        .constraints(Constraints.DIST_MEASURE_TYPE)
                        .defaultValue(AllPredefinedLayers.L2)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.MAX_STORED_SAMPLES_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE_OR_MINUS_ONE)
                        .defaultValue(-1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.LOCAL_ADAPTION_K_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERY_NET_DIR_NAME)
                        .constraints(Constraints.STRING)
                        .defaultValue(-1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERY_NET_PREFIX_NAME)
                        .constraints(Constraints.STRING)
                        .defaultValue(-1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
