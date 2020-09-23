/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Dot extends PredefinedLayerDeclaration {

    private Dot() {
        super(AllPredefinedLayers.DOT_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(layer.getInputTypes().get(0).getChannels())
                .height(layer.getInputTypes().get(1).getHeight())
                .width(1)
                .elementType("-oo", "oo")
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
        errorIfInputWidthIsInvalid(inputTypes, layer, 1);

        if (layer.getInputTypes().get(0).getHeight().intValue() != layer.getInputTypes().get(1).getChannels().intValue()) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. Dot cannot be applied to input 0 with height " +
                            layer.getInputTypes().get(0).getHeight() + " and input 1 with channel size " + layer.getInputTypes().get(1).getChannels()
                    , layer.getSourcePosition());
        }
    }

    public static Dot create(){
        Dot declaration = new Dot();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
