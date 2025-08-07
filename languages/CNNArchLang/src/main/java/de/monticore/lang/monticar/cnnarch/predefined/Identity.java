/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */

package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class Identity extends PredefinedLayerDeclaration {

    private Identity() {
        super(AllPredefinedLayers.IDENTITY_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        ArchTypeSymbol symbol = layer.getInputTypes().get(0);
        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(symbol.getChannels())
                        .height(symbol.getHeight())
                        .width(symbol.getWidth())
                        .elementType(symbol.getDomain())
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Identity create() {
        Identity declaration = new Identity();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}





