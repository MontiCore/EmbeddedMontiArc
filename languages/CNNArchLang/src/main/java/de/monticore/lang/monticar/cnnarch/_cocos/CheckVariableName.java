/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class CheckVariableName extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureElementSymbol sym) {
        if (sym instanceof VariableSymbol) {
            checkVariable((VariableSymbol) sym);
        }
    }

    public void checkVariable(VariableSymbol variable) {
        Collection<VariableDeclarationSymbol> declarations
                = variable.getArchitecture().getSpannedScope().resolveMany(variable.getName(), VariableDeclarationSymbol.KIND);

        if (declarations.isEmpty()) {
            Log.error("0" + ErrorCodes.UNKNOWN_VARIABLE_NAME + " Unknown variable name. " +
                            "Variable '" + variable.getName() + "' does not exist. "
                      , variable.getSourcePosition());
        }
    }

}
