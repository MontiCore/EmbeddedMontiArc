/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.PredefinedLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;
import org.jscience.mathematics.number.Rational;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Add extends PredefinedLayerDeclaration {

    private Add() {super(AllPredefinedLayers.ADD_NAME);}

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        List<String> range = computeStartAndEndValue(layer.getInputTypes(), Rational::plus, Rational::plus);

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .channels(layer.getInputTypes().get(0).getChannels())
                .height(layer.getInputTypes().get(0).getHeight())
                .width(layer.getInputTypes().get(0).getWidth())
                .elementType(range.get(0), range.get(1))
                .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputIsEmpty(inputTypes, layer);
        errorIfMultipleInputShapesAreNotEqual(inputTypes, layer, HandlingSingleInputs.IGNORED);
    }

    public static Add create(){
        Add declaration = new Add();
        declaration.setParameters(new ArrayList<>());
        return declaration;
    }
}
