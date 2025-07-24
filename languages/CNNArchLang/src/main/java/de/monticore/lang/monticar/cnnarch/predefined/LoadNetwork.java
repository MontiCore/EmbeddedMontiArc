/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */

package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class LoadNetwork extends PredefinedLayerDeclaration {

    private LoadNetwork() {
        super(AllPredefinedLayers.LOAD_NETWORK_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        Optional<List<Integer>> optValue = layer.getIntOrIntTupleValues(AllPredefinedLayers.OUTPUT_SHAPE_NAME);
        Optional<String> optLowerValue = layer.getStringValue(AllPredefinedLayers.ELEMENT_TYPE_LOWER_BOUND);
        Optional<String> optUpperValue = layer.getStringValue(AllPredefinedLayers.ELEMENT_TYPE_UPPER_BOUND);


        List<Integer> shapeList = Arrays.asList(1, 1, 1);

        if (optValue.isPresent()) {
            List<Integer> outputShape = optValue.get();

            for (int i = 0; i < outputShape.size() && i < 3; i++) {
                shapeList.set(i, outputShape.get(i));
            }
        }
        if (optLowerValue.isPresent() && optUpperValue.isPresent())
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(shapeList.get(0))
                    .height(shapeList.get(1))
                    .width(shapeList.get(2))
                    .elementType(optLowerValue.get(), optUpperValue.get())
                    .build());
        else return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(shapeList.get(0))
                .height(shapeList.get(1))
                .width(shapeList.get(2))
                .elementType("-oo", "oo")
                .build());

    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
    }

    public static LoadNetwork create() {
        LoadNetwork declaration = new LoadNetwork();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NETWORK_DIR_NAME)
                        .constraints(Constraints.PATH_TAG_OR_PATH)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NETWORK_PREFIX_NAME)
                        .constraints(Constraints.STRING)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NUM_INPUTS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.OUTPUT_SHAPE_NAME)
                        .constraints(Constraints.INTEGER_OR_INTEGER_TUPLE, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.TRAINABLE)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(true)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.ELEMENT_TYPE_LOWER_BOUND)
                        .constraints(Constraints.STRING)
                        .defaultValue("-oo")
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.ELEMENT_TYPE_UPPER_BOUND)
                        .constraints(Constraints.STRING)
                        .defaultValue("oo")
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
