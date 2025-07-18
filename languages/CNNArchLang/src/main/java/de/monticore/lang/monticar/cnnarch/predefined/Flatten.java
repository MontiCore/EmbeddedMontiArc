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

public class Flatten extends PredefinedLayerDeclaration {

    private Flatten() {
        super(AllPredefinedLayers.FLATTEN_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        if (inputTypes.get(0).getDepthIndex() == -1){
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .height(1)
                    .width(1)
                    .channels(layer.getInputTypes().get(0).getHeight()
                            * layer.getInputTypes().get(0).getWidth()
                            * layer.getInputTypes().get(0).getChannels())
                    .elementType(layer.getInputTypes().get(0).getDomain())
                    .build());
        } else {
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .height(1)
                    .width(1)
                    .depth(1)
                    .channels(layer.getInputTypes().get(0).getHeight()
                            * layer.getInputTypes().get(0).getWidth()
                            * layer.getInputTypes().get(0).getChannels()
                            * layer.getInputTypes().get(0).getDepth())
                    .elementType(layer.getInputTypes().get(0).getDomain())
                    .build());
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Flatten create(){
        Flatten declaration = new Flatten();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
