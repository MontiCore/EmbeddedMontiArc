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

public class Reshape extends PredefinedLayerDeclaration {

    private Reshape() {
        super(AllPredefinedLayers.RESHAPE_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        List<Integer> shape = layer.getIntTupleValue(AllPredefinedLayers.SHAPE_NAME).get();

        int channels = -1;
        int height = -1;
        int width = -1;
        int depth = -1;
        
        if (shape.size() >= 4) {
             depth = shape.get(1);
             height = shape.get(2);
             width = shape.get(3);
             channels = shape.get(0);
        } else {
        
            if (shape.size() >= 3) {
                width = shape.get(2);
            }

            if (shape.size() >= 2) {
                height = shape.get(1);
            }

            if (shape.size() >= 1) {
                channels = shape.get(0);
            } else {
                Log.error("0" + ErrorCodes.ILLEGAL_PARAMETER_VALUE + "\"Shape\" argument needs to contain at least one entry"
                        , layer.getSourcePosition());
            }
        }
        int totalSize = layer.getInputTypes().get(0).getChannels() * layer.getInputTypes().get(0).getHeight() * layer.getInputTypes().get(0).getWidth() * layer.getInputTypes().get(0).getDepth();
        int newTotalSize = shape.stream().reduce(1, (x, y) -> x * y);

        if (totalSize != newTotalSize && newTotalSize != 0) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + "The input of Reshape layer cannot be reshaped to the given shape. "
                    + "Source and target shape have a different amount of total values", layer.getSourcePosition());
        }

        if (newTotalSize != 0) {
            if (depth != -1) {
                return Collections.singletonList(
                        new ArchTypeSymbol.Builder()
                                .channels(channels)
                                .height(height)
                                .width(width)
                                .depth(depth)
                                .elementType(layer.getInputTypes().get(0).getDomain())
                                .build());
            } else if (width != -1) {
                return Collections.singletonList(
                        new ArchTypeSymbol.Builder()
                                .channels(channels)
                                .height(height)
                                .width(width)
                                .elementType(layer.getInputTypes().get(0).getDomain())
                                .build());
            } else if (height != -1) {
                return Collections.singletonList(
                        new ArchTypeSymbol.Builder()
                                .channels(channels)
                                .height(height)
                                .elementType(layer.getInputTypes().get(0).getDomain())
                                .build());
            } else{
                return Collections.singletonList(
                        new ArchTypeSymbol.Builder()
                                .channels(channels)
                                .elementType(layer.getInputTypes().get(0).getDomain())
                                .build());
            }
        }
        return Collections.emptyList();
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Reshape create(){
        Reshape declaration = new Reshape();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SHAPE_NAME)
                        .constraints(Constraints.INTEGER_TUPLE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
