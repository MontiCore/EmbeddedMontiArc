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

public class EpisodicMemory extends PredefinedLayerDeclaration {

    private EpisodicMemory() {
        super(AllPredefinedLayers.EPISODIC_MEMORY_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        List<ArchTypeSymbol> outputShapes  = new ArrayList<>(layer.getInputTypes().size());
        for (int i = 0; i < layer.getInputTypes().size(); i++) {
            ArchTypeSymbol inputShape = layer.getInputTypes().get(i);
            int inputHeight = inputShape.getHeight();
            int inputWidth = inputShape.getWidth();
            int inputChannels = inputShape.getChannels();

            outputShapes.add(new ArchTypeSymbol.Builder()
                    .height(inputHeight)
                    .width(inputWidth)
                    .channels(inputChannels)
                    .elementType(layer.getInputTypes().get(i).getDomain())
                    .build());
        }
        return outputShapes;
    }
    
    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
    }

    public static EpisodicMemory create(){
        EpisodicMemory declaration = new EpisodicMemory();
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
                        .name(AllPredefinedLayers.USE_LOCAL_ADAPTATION_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(true)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.LOCAL_ADAPTATION_GRADIENT_STEPS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.LOCAL_ADAPTATION_K_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.MEMORY_STORE_PROB_NAME)
                        .constraints(Constraints.NUMBER, Constraints.BETWEEN_ZERO_AND_ONE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.MAX_STORED_SAMPLES_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE_OR_MINUS_ONE)
                        .defaultValue(-1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.MEMORY_REPLACEMENT_STRATEGY_NAME)
                        .constraints(Constraints.MEMORY_REPLACEMENT_STRATEGY_TYPE)
                        .defaultValue(AllPredefinedLayers.REPLACE_OLDEST)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERY_NET_DIR_NAME)
                        .constraints(Constraints.PATH_TAG_OR_PATH)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERY_NET_PREFIX_NAME)
                        .constraints(Constraints.STRING)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERY_NET_NUM_INPUTS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
