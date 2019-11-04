/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Reshape extends PredefinedLayerDeclaration {
    public Reshape() {
        super(AllPredefinedLayers.RESHAPE_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        List<Integer> shape = layer.getIntTupleValue(AllPredefinedLayers.SHAPE_NAME).get();
        Collections.reverse(shape);

        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(shape.size() < 3 ? 0 : shape.get(2))
                        .height(shape.size() < 2 ? 0 : shape.get(1))
                        .width(shape.size() < 1 ? 0 : shape.get(0))
                        .elementType("0", "oo")
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

    }

    public static Reshape create(){
        Reshape reshape = new Reshape();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.SHAPE_NAME)
                        .constraints(Constraints.INTEGER_TUPLE)
                        .defaultValue(-2)
                        .build()));
        reshape.setParameters(parameters);
        return reshape;
    }
}
