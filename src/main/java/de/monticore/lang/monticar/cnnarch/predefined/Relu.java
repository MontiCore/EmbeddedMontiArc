/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.PredefinedLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Relu extends PredefinedLayerDeclaration {

    private Relu() {
        super(AllPredefinedLayers.RELU_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(layer.getInputTypes().get(0).getChannels())
                        .height(layer.getInputTypes().get(0).getHeight())
                        .width(layer.getInputTypes().get(0).getWidth())
                        .elementType("0", "oo")
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Relu create(){
        Relu declaration = new Relu();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
