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
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Split extends PredefinedLayerDeclaration {

    private Split() {
        super(AllPredefinedLayers.SPLIT_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        ArchTypeSymbol inputShape = layer.getInputTypes().get(0);
        int numberOfSplits = layer.getIntValue(AllPredefinedLayers.NUM_SPLITS_NAME).get();
        int inputHeight = inputShape.getHeight();
        int inputWidth = inputShape.getWidth();
        int inputChannels = inputShape.getChannels();

        int outputChannels = inputChannels / numberOfSplits;
        int outputChannelsLast = inputChannels - (numberOfSplits-1) * outputChannels;

        List<ArchTypeSymbol> outputShapes  = new ArrayList<>(numberOfSplits);
        for (int i = 0; i < numberOfSplits; i++){
            if (i == numberOfSplits - 1) {
                outputShapes.add(new ArchTypeSymbol.Builder()
                        .height(inputHeight)
                        .width(inputWidth)
                        .channels(outputChannelsLast)
                        .elementType(layer.getInputTypes().get(0).getDomain())
                        .build());
            } else {
                outputShapes.add(new ArchTypeSymbol.Builder()
                        .height(inputHeight)
                        .width(inputWidth)
                        .channels(outputChannels)
                        .elementType(layer.getInputTypes().get(0).getDomain())
                        .build());
            }
        }
        return outputShapes;
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        if (inputTypes.size() == 1) {
            int inputChannels = inputTypes.get(0).getChannels();
            int numberOfSplits = layer.getIntValue(AllPredefinedLayers.NUM_SPLITS_NAME).get();

            if (inputChannels < numberOfSplits){
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                                "The number of input channels " +  inputChannels +
                                " is smaller than the number of splits " + numberOfSplits + "."
                        , layer.getSourcePosition());
            }
        }
        else {
            errorIfInputSizeIsNotOne(inputTypes, layer);
        }
    }

    public static Split create(){
        Split declaration = new Split();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NUM_SPLITS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
