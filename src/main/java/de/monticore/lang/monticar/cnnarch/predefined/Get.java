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
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Get extends PredefinedLayerDeclaration {

    private Get() {
        super(AllPredefinedLayers.GET_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int index = layer.getIntValue(AllPredefinedLayers.INDEX_NAME).get();
        if (index < layer.getInputTypes().size()){
            return Collections.singletonList(layer.getInputTypes().get(index));
        }
        else {
            if (layer.getInputTypes().isEmpty()){
                return layer.getInputTypes();
            }
            else {
                return Collections.singletonList(layer.getInputTypes().get(0));
            }
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int index = layer.getIntValue(AllPredefinedLayers.INDEX_NAME).get();
        if (inputTypes.size() <= index){
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Stream index out of bound. " +
                            "The selected input stream has the index " + index +
                            " but there are only " + inputTypes.size() + " input streams."
                    , layer.getSourcePosition());
        }
    }

    public static Get create(){
        Get declaration = new Get();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.INDEX_NAME)
                        .constraints(Constraints.INTEGER, Constraints.NON_NEGATIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
