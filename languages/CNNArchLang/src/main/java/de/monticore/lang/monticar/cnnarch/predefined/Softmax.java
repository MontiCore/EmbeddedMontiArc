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
import java.util.Collections;
import java.util.List;

public class Softmax extends PredefinedLayerDeclaration {

    private Softmax() {
        super(AllPredefinedLayers.SOFTMAX_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(layer.getInputTypes().get(0).getChannels())
                        .height(layer.getInputTypes().get(0).getHeight())
                        .width(layer.getInputTypes().get(0).getWidth())
                        .elementType("0", "1")
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Softmax create(){
        Softmax declaration = new Softmax();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.AXIS_NAME)
                        .constraints(Constraints.INTEGER)
                        .defaultValue(-1)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
