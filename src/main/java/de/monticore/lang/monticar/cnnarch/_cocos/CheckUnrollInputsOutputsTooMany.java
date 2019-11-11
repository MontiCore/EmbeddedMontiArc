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
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.math3.geometry.spherical.oned.Arc;

import java.util.List;
import java.util.Optional;

public class CheckUnrollInputsOutputsTooMany extends CNNArchSymbolCoCo {

    @Override
    public void check(UnrollInstructionSymbol sym) {
        int countInputs = 0;

        for (ArchitectureElementSymbol input : sym.getBody().getFirstAtomicElements()) {
            if (input instanceof VariableSymbol) {
                VariableSymbol variable = (VariableSymbol) input;

                if (variable.isOutput() && variable.getType() == VariableSymbol.Type.IO) {
                    ++countInputs;
                }
            }
        }

        if (countInputs > 1) {
            Log.error("0" + ErrorCodes.UNROLL_INPUTS_TOO_MANY + " Only one input is allowed for timed constructs."
                    , sym.getSourcePosition());
        }

        int countOutputs = 0;

        for (ArchitectureElementSymbol output : sym.getBody().getLastAtomicElements()) {
            if (output instanceof VariableSymbol) {
                VariableSymbol variable = (VariableSymbol) output;

                if (variable.isOutput() && variable.getType() == VariableSymbol.Type.IO) {
                    ++countOutputs;
                }
            }
        }

        if (countOutputs > 1) {
            Log.error("0" + ErrorCodes.UNROLL_OUTPUTS_TOO_MANY + " Only one output is allowed for timed constructs."
                    , sym.getSourcePosition());
        }
    }
}
