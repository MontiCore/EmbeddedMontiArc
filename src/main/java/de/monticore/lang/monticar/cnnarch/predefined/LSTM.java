/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
