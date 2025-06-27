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
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Embedding extends PredefinedLayerDeclaration {

    private Embedding() {
        super(AllPredefinedLayers.EMBEDDING_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int outputDim = layer.getIntValue(AllPredefinedLayers.OUTPUT_DIM_NAME).get();

        ArchTypeSymbol inputType = layer.getInputTypes().get(0);

        if (inputType.getHeight() == 1) {
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(inputType.getChannels())
                    .height(outputDim)
                    .elementType("-oo", "oo")
                    .build());
        }
        else {
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(inputType.getChannels())
                    .height(inputType.getHeight())
                    .width(outputDim)
                    .elementType("-oo", "oo")
                    .build());
        }
    }

    private static void inferInputDim(LayerSymbol layer) {
        ASTElementType domain = layer.getInputTypes().get(0).getDomain();

        // Only infer when not already done and upper limit is available
        if (layer.getIntValue(AllPredefinedLayers.INPUT_DIM_NAME).get() == 0
                && domain.isPresentRange()
                && !domain.getRange().hasNoUpperLimit()) {
            int inputDim = domain.getRange().getEndValue().intValue() + 1;

            layer.setIntValue(AllPredefinedLayers.INPUT_DIM_NAME, inputDim);
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        inferInputDim(layer);

        errorIfInputSizeIsNotOne(inputTypes, layer);

        // Only up to 3 dimensions are supported so the input needs to be at maximum 2-dimensional as the output has one
        // more dimension than the output
        errorIfInputWidthIsInvalid(inputTypes, layer, 1);

        int inputDim = layer.getIntValue(AllPredefinedLayers.INPUT_DIM_NAME).get();

        if (inputDim == 0) {
            Log.error("0" + ErrorCodes.MISSING_ARGUMENT + " Missing argument. The argument 'input_dim' is in this case required. ", layer.getSourcePosition());
        }

        ASTElementType domain = layer.getInputTypes().get(0).getDomain();

        if (!domain.isWholeNumber() && !domain.isNaturalNumber()) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Input must be natural. ", layer.getSourcePosition());
        }
        else if (!domain.isPresentRange() || domain.getRange().hasNoUpperLimit()) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Input range must have an upper limit. ", layer.getSourcePosition());
        }
        else if (domain.getRange().getStartValue().intValue() < 0 || domain.getRange().getEndValue().intValue() >= inputDim) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Input upper limit must be smaller than 'input_dim'. ", layer.getSourcePosition());
        }
    }

    public static Embedding create(){
        Embedding declaration = new Embedding();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.INPUT_DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(0)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.OUTPUT_DIM_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
