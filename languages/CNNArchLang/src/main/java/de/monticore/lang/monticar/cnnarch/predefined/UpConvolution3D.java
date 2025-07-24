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
import java.util.List;

public class UpConvolution3D extends PredefinedLayerDeclaration {

    private UpConvolution3D() {
        super(AllPredefinedLayers.UP_CONVOLUTION3D_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return computeUpConvOutputShape(layer.getInputTypes().get(0),
                layer,
                layer.getIntValue(AllPredefinedLayers.CHANNELS_NAME).get());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static UpConvolution3D create(){
        UpConvolution3D declaration = new UpConvolution3D();
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
                        .defaultValue(Arrays.asList(1, 1, 1))
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NOBIAS_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(false)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.TRANSPADDING_NAME)
                        .constraints(Constraints.TRANSPADDING_TYPE3D)
                        .defaultValue(AllPredefinedLayers.PADDING_SAME3D)
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
