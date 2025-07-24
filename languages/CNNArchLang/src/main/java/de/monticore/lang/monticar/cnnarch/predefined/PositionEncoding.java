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

public class PositionEncoding extends PredefinedLayerDeclaration {

    private PositionEncoding() {
        super(AllPredefinedLayers.POSITION_ENCODING_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int outputDim = layer.getIntValue(AllPredefinedLayers.OUTPUT_DIM_NAME).get();
        int maxLength = layer.getIntValue(AllPredefinedLayers.MAX_LENGTH_NAME).get();
        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(maxLength)
                .height(outputDim)
                .elementType("-oo", "oo")
                .build());

    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        ;
    }

    public static PositionEncoding create(){
        PositionEncoding declaration = new PositionEncoding();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.MAX_LENGTH_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(), 
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.OUTPUT_DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
