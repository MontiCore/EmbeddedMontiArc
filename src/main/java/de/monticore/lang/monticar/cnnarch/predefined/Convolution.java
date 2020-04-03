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
import java.util.List;

public class Convolution extends PredefinedLayerDeclaration {

    private Convolution() {
        super(AllPredefinedLayers.CONVOLUTION_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        //     TODO check here
        return computeConvAndPoolOutputShape(layer.getInputTypes().get(0),
                layer,
                layer.getIntValue(AllPredefinedLayers.CHANNELS_NAME).get());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
        errorIfInputSmallerThanKernel(inputTypes, layer);
    }

    public static Convolution create(){
        Convolution declaration = new Convolution();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.KERNEL_NAME)
                        .constraints(Constraints.INTEGER_TUPLE, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.CHANNELS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.STRIDE_NAME)
                        .constraints(Constraints.INTEGER_TUPLE, Constraints.POSITIVE)
                        .defaultValue(Arrays.asList(1, 1))
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NOBIAS_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(false)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.PADDING_NAME)
                        .constraints(Constraints.PADDING_TYPE)
                        .defaultValue(AllPredefinedLayers.PADDING_SAME)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.GROUPS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
