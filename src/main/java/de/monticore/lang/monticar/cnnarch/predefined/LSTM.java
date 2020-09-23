/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;

public class LSTM extends BaseRNN {

    private LSTM() {
        super(AllPredefinedLayers.LSTM_NAME);
    }

    @Override
    public int getArrayLength(VariableSymbol.Member member) {
        if (member == VariableSymbol.Member.STATE) {
            return 2;
        }
        else if (member == VariableSymbol.Member.NONE || member == VariableSymbol.Member.OUTPUT) {
            return 1;
        }

        return 0;
    }

    public static LSTM create() {
        LSTM declaration = new LSTM();
        declaration.setParameters(createParameters());
        return declaration;
    }
}
