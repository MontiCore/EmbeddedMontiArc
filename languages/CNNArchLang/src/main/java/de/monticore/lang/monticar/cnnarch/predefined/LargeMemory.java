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

public class LargeMemory extends PredefinedLayerDeclaration {

    private LargeMemory() {
        super(AllPredefinedLayers.LARGE_MEMORY_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        Optional<Integer> optValue = layer.getIntValue(AllPredefinedLayers.QUERY_SIZE_NAME);
            
        if (optValue.isPresent()){        
            int querySize = optValue.get();
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(1)
                .height(querySize)
                .width(1)
                .elementType("-oo", "oo")
                .build());
        }else{
            Optional<List<Integer>> optTupleValue = layer.getIntTupleValue(AllPredefinedLayers.QUERY_SIZE_NAME);
            List<Integer> list = new ArrayList<>();
            for (Object value : optTupleValue.get()) {
                list.add((Integer) value);
            }
            int listLen = list.size();
            int lastEntry = list.get(listLen-1);
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(1)
                .height(lastEntry)
                .width(1)
                .elementType("-oo", "oo")
                .build());
        }
    }
    
    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static LargeMemory create(){
        LargeMemory declaration = new LargeMemory();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SUB_KEY_SIZE_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERY_SIZE_NAME)
                        .constraints(Constraints.INTEGER_OR_INTEGER_TUPLE, Constraints.POSITIVE)
                        .defaultValue(512)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.QUERY_ACT_NAME)
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
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.VALUES_DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE_OR_MINUS_ONE)
                        .defaultValue(-1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
