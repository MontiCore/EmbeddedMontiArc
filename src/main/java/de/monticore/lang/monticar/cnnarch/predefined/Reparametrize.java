/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch.predefined;

import java.util.ArrayList;

public class Reparametrize extends Add {

    private Reparametrize() { super(AllPredefinedLayers.REPARAMETRIZE_NAME); }

    public static Reparametrize create() {
        Reparametrize layerDeclaration = new Reparametrize();
        layerDeclaration.setParameters(new ArrayList<>());
        return layerDeclaration;
    }

}
