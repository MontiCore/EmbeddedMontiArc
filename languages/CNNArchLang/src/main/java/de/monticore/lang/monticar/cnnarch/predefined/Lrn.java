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
import java.util.List;

public class Lrn extends PredefinedLayerDeclaration {

    private Lrn() {
        super(AllPredefinedLayers.LRN_NAME);
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return layer.getInputTypes();
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public static Lrn create(){
        Lrn declaration = new Lrn();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.NSIZE_NAME)
                        .constraints(Constraints.INTEGER, Constraints.NON_NEGATIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.KNORM_NAME)
                        .constraints(Constraints.NUMBER)
                        .defaultValue(2)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.ALPHA_NAME)
                        .constraints(Constraints.NUMBER)
                        .defaultValue(0.0001)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.BETA_NAME)
                        .constraints(Constraints.NUMBER)
                        .defaultValue(0.75)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
