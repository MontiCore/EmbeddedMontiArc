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

public class DotProductSelfAttention extends PredefinedLayerDeclaration {

    private DotProductSelfAttention() {
        super(AllPredefinedLayers.DOT_PRODUCT_SELF_ATTENTION_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Arrays.asList(layer.getInputTypes().get(2));
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        if (inputTypes.size() < 3) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " To few inputs. " +
                            "DotProductSelfAttentnion layer expects at least 3 Inputs: querys, keys, values and " +
                            "optionally a mask if useMask is set to true, but "
                            + inputTypes.size() + " were provided."
                    , layer.getSourcePosition());
        } else if (inputTypes.size() > 4) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " To many inputs. " +
                            "DotProductSelfAttentnion layer expects 3 or 4 Inputs: querys, keys, values and " +
                            "optionally a mask if useMask is set to true, but "
                            + inputTypes.size() + " were provided."
                    , layer.getSourcePosition());
        }
    }

    public static DotProductSelfAttention create(){
        DotProductSelfAttention declaration = new DotProductSelfAttention();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SCALE_FACTOR_NAME)
                        .constraints(Constraints.POSITIVE_OR_MINUS_ONE)
                        .defaultValue(-1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NUM_HEADS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.DIM_KEYS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE_OR_MINUS_ONE)
                        .defaultValue(-1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.DIM_VALUES_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE_OR_MINUS_ONE)
                        .defaultValue(-1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.USE_PROJ_BIAS_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(true)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.USE_MASK_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(false)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
