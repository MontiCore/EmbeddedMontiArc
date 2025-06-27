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
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Concatenate extends PredefinedLayerDeclaration {

    private Concatenate() {
        super(AllPredefinedLayers.CONCATENATE_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int channels = layer.getInputTypes().get(0).getChannels();
        int height = layer.getInputTypes().get(0).getHeight();
        int width = layer.getInputTypes().get(0).getWidth();
        if (inputTypes.get(0).getDepthIndex() > -1) {
            int depth = layer.getInputTypes().get(0).getDepth();
        }

        int axis = layer.getIntValue(AllPredefinedLayers.AXIS_NAME).get();

        List<String> range = computeStartAndEndValue(layer.getInputTypes(), (x,y) -> x.isLessThan(y) ? x : y, (x,y) -> x.isLessThan(y) ? y : x);

        List<ArchTypeSymbol> types = layer.getInputTypes();
        types.remove(0);

        if (axis == 0) {
            for (ArchTypeSymbol inputShape : types) {
                channels += inputShape.getChannels();
            }
        } else if (axis == 1) {
            for (ArchTypeSymbol inputShape : types) {
                height += inputShape.getHeight();
            }
        } else {
            for (ArchTypeSymbol inputShape : types) {
                width += inputShape.getWidth();
            }
        }
        if (inputTypes.get(0).getDepthIndex() > -1) {
            return Collections.singletonList(
                    new ArchTypeSymbol.Builder()
                        .channels(layer.getInputTypes().get(0).getChannels())
                        .height(layer.getInputTypes().get(0).getHeight())
                        .width(layer.getInputTypes().get(0).getWidth())
                        .depth(layer.getInputTypes().get(0).getDepth())
                        .elementType(range.get(0), range.get(1))
                        .build());
        } else {
            return Collections.singletonList(
                    new ArchTypeSymbol.Builder()
                        .channels(channels)
                        .height(height)
                        .width(width)
                        .elementType(range.get(0), range.get(1))
                        .build());
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);

        List<Integer> channelList = new ArrayList<>();
        List<Integer> heightList = new ArrayList<>();
        List<Integer> widthList = new ArrayList<>();
        for (ArchTypeSymbol shape : inputTypes){
            heightList.add(shape.getHeight());
            widthList.add(shape.getWidth());
            channelList.add(shape.getChannels());
        }
        int countEqualChannels = (int)channelList.stream().distinct().count();
        int countEqualHeights = (int)heightList.stream().distinct().count();
        int countEqualWidths = (int)widthList.stream().distinct().count();
        if (countEqualHeights != 1 && countEqualWidths != 1 && countEqualChannels != 1){
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                            "Concatenation of inputs with different resolutions is not possible. " +
                            "Input channels: " + Joiners.COMMA.join(channelList) + ". " +
                            "Input heights: " + Joiners.COMMA.join(heightList) + ". " +
                            "Input widths: " + Joiners.COMMA.join(widthList) + ". "
                    , layer.getSourcePosition());
        }
    }

    public static Concatenate create(){
        Concatenate declaration = new Concatenate();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.AXIS_NAME)
                        .constraints(Constraints.AXIS)
                        .defaultValue(0)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
