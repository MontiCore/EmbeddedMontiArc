/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTIntegerListValue;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;

public class CheckIntegerList implements CNNTrainASTIntegerListValueCoCo {

    @Override
    public void check(ASTIntegerListValue node) {
        for (ASTNumberWithUnit element : node.getNumberList()) {
            Double unitNumber = element.getNumber().get();
            if ((unitNumber % 1)!= 0) {
                Log.error("0" + ErrorCodes.NOT_INTEGER_CODE +" Value has to be an integer."
                        , node.get_SourcePositionStart());
            }
        }
    }

}
