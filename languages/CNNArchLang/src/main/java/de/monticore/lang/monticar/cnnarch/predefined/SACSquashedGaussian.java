/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch.predefined;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.Constraints;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.PredefinedLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;

public class SACSquashedGaussian extends PredefinedLayerDeclaration {

    private SACSquashedGaussian(){ super(AllPredefinedLayers.SAC_SQUASHED_GAUSSIAN_NAME);}

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int outputHeight = layer.getIntValue(AllPredefinedLayers.OUTPUT_DIM_NAME).get();

        List<ArchTypeSymbol> outputShapes  = new ArrayList<>(1);

        outputShapes.add(new ArchTypeSymbol.Builder() //mean_action
                .height(outputHeight)
                .width(1)
                .channels(layer.getInputTypes().get(0).getChannels())
                .elementType("-1", "1")
                .build());
        outputShapes.add(new ArchTypeSymbol.Builder() //random_action
                .height(outputHeight)
                .width(1)
                .channels(layer.getInputTypes().get(0).getChannels())
                .elementType("-1", "1")
                .build());
        outputShapes.add(new ArchTypeSymbol.Builder() //random_action_logprob
                .height(1)
                .width(1)
                .channels(layer.getInputTypes().get(0).getChannels())
                .elementType("-oo","oo")
                .build());

        return outputShapes;
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static SACSquashedGaussian create(){
        SACSquashedGaussian declaration = new SACSquashedGaussian();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.OUTPUT_DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SAC_STD_LOG_MIN_NAME)
                        .constraints(Constraints.NUMBER)
                        .defaultValue(-20)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SAC_STD_LOG_MAX_NAME)
                        .constraints(Constraints.NUMBER)
                        .defaultValue(2)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}