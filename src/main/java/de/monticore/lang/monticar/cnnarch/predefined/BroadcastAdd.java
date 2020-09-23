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
import org.jscience.mathematics.number.Rational;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BroadcastAdd extends PredefinedLayerDeclaration {

    private BroadcastAdd() {
        super(AllPredefinedLayers.BROADCAST_ADD_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        List<String> range = computeStartAndEndValue(layer.getInputTypes(), Rational::plus, Rational::plus);

        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(Math.max(layer.getInputTypes().get(0).getChannels(), layer.getInputTypes().get(1).getChannels()))
                        .height(Math.max(layer.getInputTypes().get(0).getHeight(), layer.getInputTypes().get(1).getHeight()))
                        .width(Math.max(layer.getInputTypes().get(0).getWidth(), layer.getInputTypes().get(1).getWidth()))
                        .elementType(range.get(0), range.get(1))
                        .build());

    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
    }

    public static BroadcastAdd create(){
        BroadcastAdd declaration = new BroadcastAdd();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}