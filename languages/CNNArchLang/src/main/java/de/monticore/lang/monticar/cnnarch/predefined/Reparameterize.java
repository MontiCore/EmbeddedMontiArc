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

public class Reparameterize extends PredefinedLayerDeclaration {

    private Reparameterize() { super(AllPredefinedLayers.REPARAMETERIZE_NAME); }

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
        errorIfInputIsEmpty(inputTypes,layer);
        errorIfInputNotFlattened(inputTypes,layer);
        errorIfMultipleInputShapesAreNotEqual(inputTypes, layer, HandlingSingleInputs.RESTRICTED);
    }

    public static Reparameterize create() {
        Reparameterize layerDeclaration = new Reparameterize();
        layerDeclaration.setParameters(new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.PDF_NAME)
                        .constraints(Constraints.REPARAMETERIZE_PDFS)
                        .defaultValue(AllPredefinedLayers.PDF_NORMAL)
                        .build())));
        return layerDeclaration;
    }
}
