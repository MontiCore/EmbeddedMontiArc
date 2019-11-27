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

public class Squeeze extends PredefinedLayerDeclaration {

    private Squeeze() {
        super(AllPredefinedLayers.SQUEEZE_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        List<Integer> dimensions = layer.getInputTypes().get(0).getDimensions();

        int axis = layer.getIntValue(AllPredefinedLayers.AXIS_NAME).get();

        if (axis == -1) {
            dimensions.remove(new Integer(1));
        } else {
            dimensions.remove(axis);
        }

        while (dimensions.size() < 3) {
            dimensions.add(1);
        }

        int channels = dimensions.get(0);
        int height = dimensions.get(1);
        int width = dimensions.get(2);

        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(channels)
                        .height(height)
                        .width(width)
                        .elementType(layer.getInputTypes().get(0).getDomain())
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);

        int axis = layer.getIntValue(AllPredefinedLayers.AXIS_NAME).get();

        if (axis == 0) {
            errorIfInputChannelSizeIsInvalid(inputTypes, layer, 1);
        } else if (axis == 1) {
            errorIfInputHeightIsInvalid(inputTypes, layer, 1);
        } else if (axis == 2) {
            errorIfInputWidthIsInvalid(inputTypes, layer, 1);
        }
    }

    public static Squeeze create(){
        Squeeze declaration = new Squeeze();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.AXIS_NAME)
                        .constraints(Constraints.NULLABLE_AXIS)
                        .defaultValue(-1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
