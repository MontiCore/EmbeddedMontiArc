/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

public class LSTM extends BaseRNN {

    private LSTM() {
        super(AllPredefinedLayers.LSTM_NAME);
    }

    public static LSTM create() {
        LSTM declaration = new LSTM();
        declaration.setParameters(createParameters());
        return declaration;
    }
}
