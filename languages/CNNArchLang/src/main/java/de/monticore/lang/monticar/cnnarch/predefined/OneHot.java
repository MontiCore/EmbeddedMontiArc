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
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class OneHot extends PredefinedLayerDeclaration {

    private OneHot() {
        super(AllPredefinedLayers.ONE_HOT_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(layer.getIntValue(AllPredefinedLayers.SIZE_NAME).get())
                .height(1)
                .width(1)
                .elementType("0", "1")
                .build());
    }

    private static void inferSizeFromOutput(LayerSymbol layer){
        // Only infer when not already done and next element is output
        if (layer.getIntValue(AllPredefinedLayers.SIZE_NAME).get() == 0
                && layer.getOutputElement().isPresent()
                && layer.getOutputElement().get().isOutput()) {
            int outputChannels = ((VariableSymbol) layer.getOutputElement().get()).getIoDeclaration().getType().getChannels();

            layer.setIntValue(AllPredefinedLayers.SIZE_NAME, outputChannels);
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        inferSizeFromOutput(layer);

        errorIfInputSizeIsNotOne(inputTypes, layer);
        errorIfInputChannelSizeIsInvalid(inputTypes, layer, 1);
        errorIfInputHeightIsInvalid(inputTypes, layer, 1);
        errorIfInputWidthIsInvalid(inputTypes, layer, 1);

        // Check range of input
        int size = layer.getIntValue(AllPredefinedLayers.SIZE_NAME).get();

        if (size == 0) {
            Log.error("0" + ErrorCodes.MISSING_ARGUMENT + " Missing argument. The argument 'size' is in this case required. "
                    , layer.getSourcePosition());

        }

        ASTElementType domain = inputTypes.get(0).getDomain();

        if (!domain.isNaturalNumber() && !domain.isWholeNumber()) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Input needs to be natural or whole. "
                      , layer.getSourcePosition());
        }
        else {
            if (!domain.getRangeOpt().isPresent()) {
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Range is missing. "
                          , layer.getSourcePosition());
            }
            else {
                ASTRange range = domain.getRangeOpt().get();

                if (!range.getMin().getNumber().isPresent() || !range.getMax().getNumber().isPresent()) {
                    Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Minimum or maximum is missing. "
                              , layer.getSourcePosition());
                }
                else {
                    double min = range.getMin().getNumber().get();
                    double max = range.getMax().getNumber().get();

                    // Check if minimum >= 0
                    if (min < 0) {
                        Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: Minimum needs to be bigger than 0. "
                                  , layer.getSourcePosition());
                    }

                    // Check if maximum < size
                    if (max >= size) {
                        Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " Invalid layer input domain: "
                                  + "Maximum needs to be smaller than size " + size + ". "
                                  , layer.getSourcePosition());
                    }
                }
            }
        }
    }

    public static OneHot create(){
        OneHot declaration = new OneHot();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SIZE_NAME)
                        .constraints(Constraints.POSITIVE, Constraints.INTEGER)
                        .defaultValue(0)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
