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

public class ExpandDims extends PredefinedLayerDeclaration {

    private ExpandDims() {
        super(AllPredefinedLayers.EXPAND_DIMS_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int axis = layer.getIntValue(AllPredefinedLayers.AXIS_NAME).get();

        int channels = layer.getInputTypes().get(0).getChannels();
        int height = layer.getInputTypes().get(0).getHeight();
        int width = layer.getInputTypes().get(0).getWidth();

        if (axis == 0) {
            width = height;
            height = channels;
            channels = 1;
        } else if (axis == 1) {
            width = height;
            height = 1;
        }

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(channels)
                .height(height)
                .width(width)
                .elementType(layer.getInputTypes().get(0).getDomain())
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
        errorIfInputWidthIsInvalid(inputTypes, layer, 1);
    }

    public static ExpandDims create(){
        ExpandDims declaration = new ExpandDims();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.AXIS_NAME)
                        .constraints(Constraints.AXIS_WITHOUT_2)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
