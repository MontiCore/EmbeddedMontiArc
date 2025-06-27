/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

public class RNN extends BaseRNN {

    private RNN() {
        super(AllPredefinedLayers.RNN_NAME);
    }

    public static RNN create() {
        RNN declaration = new RNN();
        declaration.setParameters(createParameters());
        return declaration;
    }
}
