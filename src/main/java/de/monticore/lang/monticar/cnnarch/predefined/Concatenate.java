/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.PredefinedLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Concatenate extends PredefinedLayerDeclaration {

    private Concatenate() {
        super(AllPredefinedLayers.CONCATENATE_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        int height = layer.getInputTypes().get(0).getHeight();
        int width = layer.getInputTypes().get(0).getWidth();
        int channels = 0;
        for (ArchTypeSymbol inputShape : layer.getInputTypes()) {
            channels += inputShape.getChannels();
        }

        List<String> range = computeStartAndEndValue(layer.getInputTypes(), (x,y) -> x.isLessThan(y) ? x : y, (x,y) -> x.isLessThan(y) ? y : x);

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(channels)
                .height(height)
                .width(width)
                .elementType(range.get(0), range.get(1))
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        if (!inputTypes.isEmpty()) {
            List<Integer> heightList = new ArrayList<>();
            List<Integer> widthList = new ArrayList<>();
            for (ArchTypeSymbol shape : inputTypes){
                heightList.add(shape.getHeight());
                widthList.add(shape.getWidth());
            }
            int countEqualHeights = (int)heightList.stream().distinct().count();
            int countEqualWidths = (int)widthList.stream().distinct().count();
            if (countEqualHeights != 1 || countEqualWidths != 1){
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                                "Concatenation of inputs with different resolutions is not possible. " +
                                "Input heights: " + Joiners.COMMA.join(heightList) + ". " +
                                "Input widths: " + Joiners.COMMA.join(widthList) + ". "
                        , layer.getSourcePosition());
            }
        }
        else {
            errorIfInputIsEmpty(inputTypes, layer);
        }
    }

    public static Concatenate create(){
        Concatenate declaration = new Concatenate();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
