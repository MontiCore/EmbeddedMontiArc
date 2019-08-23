/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

public class GRU extends BaseRNN {

    private GRU() {
        super(AllPredefinedLayers.GRU_NAME);
    }

    public static GRU create() {
        GRU declaration = new GRU();
        declaration.setParameters(createParameters());
        return declaration;
    }
}
