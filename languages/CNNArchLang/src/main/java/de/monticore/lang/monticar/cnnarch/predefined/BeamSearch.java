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
import java.util.List;

public class BeamSearch extends UnrollDeclarationSymbol {

    private BeamSearch() {
        super(AllPredefinedLayers.BEAMSEARCH_NAME);
    }

    public static BeamSearch create(){
        BeamSearch declaration = new BeamSearch();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.MAX_LENGTH_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.WIDTH_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build()));
        declaration.setParameters(parameters);
        return declaration;
    }
}
