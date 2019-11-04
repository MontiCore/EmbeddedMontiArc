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

public class GlobalPooling extends PredefinedLayerDeclaration {

    protected GlobalPooling() {
        super(AllPredefinedLayers.GLOBAL_POOLING_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(1)
                .width(1)
                .channels(layer.getInputTypes().get(0).getChannels())
                .elementType(layer.getInputTypes().get(0).getDomain())
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static GlobalPooling create(){
        GlobalPooling declaration = new GlobalPooling();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.POOL_TYPE_NAME)
                        .constraints(Constraints.POOL_TYPE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
