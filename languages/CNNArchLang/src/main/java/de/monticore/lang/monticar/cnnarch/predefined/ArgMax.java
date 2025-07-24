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
import java.util.Collections;
import java.util.List;

public class ArgMax extends PredefinedLayerDeclaration {

    private ArgMax() {
        super(AllPredefinedLayers.ARG_MAX_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        ArchTypeSymbol inputType = layer.getInputTypes().get(0);

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(1)
                .height(1)
                .width(1)
                .elementType("Z", "0", String.valueOf(inputType.getChannels() - 1))
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);

        errorIfInputHeightIsInvalid(inputTypes, layer, 1);
        errorIfInputWidthIsInvalid(inputTypes, layer, 1);
    }

    public static ArgMax create(){
        ArgMax declaration = new ArgMax();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
