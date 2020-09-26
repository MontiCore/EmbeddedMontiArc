/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTIntegerValue;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckInteger implements CNNTrainASTIntegerValueCoCo {

    @Override
    public void check(ASTIntegerValue node) {
        Double unitNumber = node.getNumberWithUnit().getNumber().get();
        if ((unitNumber % 1)!= 0) {
            Log.error("0" + ErrorCodes.NOT_INTEGER_CODE +" Value has to be an integer."
                    , node.get_SourcePositionStart());
        }
    }

}
