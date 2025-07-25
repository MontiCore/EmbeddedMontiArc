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

public class SwapAxes extends PredefinedLayerDeclaration {

    private SwapAxes() {
        super(AllPredefinedLayers.SWAPAXES_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        int firstAxis = layer.getIntTupleValue(AllPredefinedLayers.AXES_NAME).get().get(0);
        int secondAxis = layer.getIntTupleValue(AllPredefinedLayers.AXES_NAME).get().get(1);

        if((firstAxis == 0 && secondAxis == 1) || (firstAxis == 1 && secondAxis == 0)){
            return Collections.singletonList(
                    new ArchTypeSymbol.Builder()
                            .channels(layer.getInputTypes().get(0).getHeight())
                            .height(layer.getInputTypes().get(0).getChannels())
                            .width(layer.getInputTypes().get(0).getWidth())
                            .elementType(layer.getInputTypes().get(0).getDomain())
                            .build());
        }else if((firstAxis == 0 && secondAxis == 2) || (firstAxis == 2 && secondAxis == 0)){
            return Collections.singletonList(
                    new ArchTypeSymbol.Builder()
                            .channels(layer.getInputTypes().get(0).getWidth())
                            .height(layer.getInputTypes().get(0).getHeight())
                            .width(layer.getInputTypes().get(0).getChannels())
                            .elementType(layer.getInputTypes().get(0).getDomain())
                            .build());
        }else if((firstAxis == 1 && secondAxis == 2) || (firstAxis == 2 && secondAxis == 1)){
            return Collections.singletonList(
                    new ArchTypeSymbol.Builder()
                            .channels(layer.getInputTypes().get(0).getChannels())
                            .height(layer.getInputTypes().get(0).getWidth())
                            .width(layer.getInputTypes().get(0).getHeight())
                            .elementType(layer.getInputTypes().get(0).getDomain())
                            .build());
        }else{
            if ((firstAxis < 0 || firstAxis > 2 || secondAxis < 0 || secondAxis > 2)){
                Log.error("0" + ErrorCodes.ILLEGAL_PARAMETER_VALUE + " Illegal value for parameter axes. Values must be between 0 and 2"
                        , layer.getSourcePosition());
            }
            return new ArrayList<>();
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static SwapAxes create(){
        SwapAxes declaration = new SwapAxes();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.AXES_NAME)
                        .constraints(Constraints.INTEGER_TUPLE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}