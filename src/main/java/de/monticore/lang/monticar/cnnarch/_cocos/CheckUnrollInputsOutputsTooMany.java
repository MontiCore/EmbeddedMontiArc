/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
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
