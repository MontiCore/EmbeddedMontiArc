/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.PredefinedLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ConcatLabels extends PredefinedLayerDeclaration {
    private ConcatLabels(){ super(AllPredefinedLayers.CONCAT_LABELS_NAME);}

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        if (layer.getInputTypes().isEmpty()){
            return layer.getInputTypes();
        }
        else {
            return Collections.singletonList(layer.getInputTypes().get(0));
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static ConcatLabels create(){
        ConcatLabels declaration = new ConcatLabels();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
