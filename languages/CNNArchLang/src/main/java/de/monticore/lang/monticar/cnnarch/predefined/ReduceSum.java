/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ReduceSum extends PredefinedLayerDeclaration {

    private ReduceSum() {
        super(AllPredefinedLayers.REDUCE_SUM_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int axis = layer.getIntValue(AllPredefinedLayers.AXIS_NAME).get();

        int channels = layer.getInputTypes().get(0).getChannels();
        int height = layer.getInputTypes().get(0).getHeight();
        int width = layer.getInputTypes().get(0).getWidth();

        if (axis == 0) {
            height = 1;
        } else if (axis == 1) {
            width = 1;
        } else {
            channels = 1;
            height = 1;
            width = 1;
        }

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(channels)
                    .height(height)
                    .width(width)
                    .elementType("-oo", "oo")
                    .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
    }

    public static ReduceSum create(){
        ReduceSum declaration = new ReduceSum();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.AXIS_NAME)
                        .constraints(Constraints.NULLABLE_AXIS_WITHOUT_2)
                        .defaultValue(-1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
