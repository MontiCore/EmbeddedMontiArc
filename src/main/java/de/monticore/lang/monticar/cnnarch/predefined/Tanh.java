/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.PredefinedLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Tanh extends PredefinedLayerDeclaration {

    private Tanh() {
        super(AllPredefinedLayers.TANH_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(layer.getInputTypes().get(0).getChannels())
                        .height(layer.getInputTypes().get(0).getHeight())
                        .width(layer.getInputTypes().get(0).getWidth())
                        .elementType("-1", "1")
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Tanh create(){
        Tanh declaration = new Tanh();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
