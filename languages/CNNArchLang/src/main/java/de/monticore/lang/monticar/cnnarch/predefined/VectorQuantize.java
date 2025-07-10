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

public class VectorQuantize extends PredefinedLayerDeclaration {

    private VectorQuantize(){ super(AllPredefinedLayers.VECTOR_QUANTIZE_NAME);}

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(layer.getInputTypes().get(0).getChannels())
                .height(layer.getInputTypes().get(0).getHeight())
                .width(layer.getInputTypes().get(0).getWidth())
                .elementType("-oo", "oo")
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static VectorQuantize create(){
        VectorQuantize declaration = new VectorQuantize();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NUM_EMBEDDINGS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.BETA_NAME)
                        .constraints(Constraints.NUMBER)
                        .defaultValue(0.25)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
