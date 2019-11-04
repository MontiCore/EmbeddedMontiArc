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
import org.jscience.mathematics.number.Rational;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Add extends PredefinedLayerDeclaration {

    private Add() {
        super(AllPredefinedLayers.ADD_NAME);
    }

    @Override
    public boolean isTrainable() {
        return false;
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        List<String> range = computeStartAndEndValue(layer.getInputTypes(), Rational::plus, Rational::plus);

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(layer.getInputTypes().get(0).getChannels())
                .height(layer.getInputTypes().get(0).getHeight())
                .width(layer.getInputTypes().get(0).getWidth())
                .elementType(range.get(0), range.get(1))
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
        if (inputTypes.size() == 1){
            Log.warn("Add layer has only one input stream. Layer can be removed." , layer.getSourcePosition());
        }
        else if (inputTypes.size() > 1){
            List<Integer> heightList = new ArrayList<>();
            List<Integer> widthList = new ArrayList<>();
            List<Integer> channelsList = new ArrayList<>();
            for (ArchTypeSymbol shape : inputTypes){
                heightList.add(shape.getHeight());
                widthList.add(shape.getWidth());
                channelsList.add(shape.getChannels());
            }
            int countEqualHeights = (int)heightList.stream().distinct().count();
            int countEqualWidths = (int)widthList.stream().distinct().count();
            int countEqualNumberOfChannels = (int)channelsList.stream().distinct().count();
            if (countEqualHeights != 1 || countEqualWidths != 1 || countEqualNumberOfChannels != 1){
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                                "Shapes of all input streams must be equal. " +
                                "Input heights: " + Joiners.COMMA.join(heightList) + ". " +
                                "Input widths: " + Joiners.COMMA.join(widthList) + ". " +
                                "Number of input channels: " + Joiners.COMMA.join(channelsList) + ". "
                        , layer.getSourcePosition());
            }
        }
    }

    public static Add create(){
        Add declaration = new Add();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
