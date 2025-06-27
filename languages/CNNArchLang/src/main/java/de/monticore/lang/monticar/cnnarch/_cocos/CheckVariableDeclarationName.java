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

import java.util.HashSet;
import java.util.Set;

public class CheckVariableDeclarationName extends CNNArchSymbolCoCo {

    Set<String> variableNames = new HashSet<>();

    @Override
    public void check(VariableDeclarationSymbol sym) {
        String name = sym.getName();

        if (name.isEmpty() || name.endsWith("_")) {
            Log.error("0" + ErrorCodes.ILLEGAL_NAME + " Illegal variable name. " +
                            "Variable names cannot end with \"_\"",
                      sym.getSourcePosition());
        }

        if (variableNames.contains(name)) {
            Log.error("0" + ErrorCodes.DUPLICATED_NAME + " Duplicated variable name. " +
                            "The name '" + name + "' is already used.",
                      sym.getSourcePosition());
        }
        else {
            variableNames.add(name);
        }
    }
}
