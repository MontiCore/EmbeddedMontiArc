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
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class LeakyRelu extends PredefinedLayerDeclaration {

    private LeakyRelu() { super(AllPredefinedLayers.LEAKY_RELU_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        if (inputTypes.get(0).getDepthIndex() == -1) {
            return Collections.singletonList(
                    new ArchTypeSymbol.Builder()
                            .channels(layer.getInputTypes().get(0).getChannels())
                            .height(layer.getInputTypes().get(0).getHeight())
                            .width(layer.getInputTypes().get(0).getWidth())
                            .elementType("0", "oo")
                            .build());
        }
        else {
            return Collections.singletonList(
                    new ArchTypeSymbol.Builder()
                            .channels(layer.getInputTypes().get(0).getChannels())
                            .height(layer.getInputTypes().get(0).getHeight())
                            .width(layer.getInputTypes().get(0).getWidth())
                            .depth(layer.getInputTypes().get(0).getDepth())
                            .elementType("0", "oo")
                            .build());
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static LeakyRelu create(){
        LeakyRelu declaration = new LeakyRelu();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.ALPHA_NAME)
                        .constraints(Constraints.POSITIVE)
                        .defaultValue(0)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
