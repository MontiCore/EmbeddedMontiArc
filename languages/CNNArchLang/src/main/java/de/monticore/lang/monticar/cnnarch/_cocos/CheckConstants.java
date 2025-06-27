/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ConstantSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class CheckConstants extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureElementSymbol sym) {
        if (sym instanceof ConstantSymbol) {
            checkConstant((ConstantSymbol) sym);
        }
    }

    public void checkConstant(ConstantSymbol constant) {
        Optional<Boolean> isInt = constant.getExpression().isInt();

        if (!isInt.isPresent() || !isInt.get()) {
            Log.error("0" + ErrorCodes.INVALID_CONSTANT + " Invalid constant, only integers allowed. ",
                    constant.getSourcePosition());
        }
    }
}
