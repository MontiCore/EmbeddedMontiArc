/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._ast.ASTArchValueRange;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckRangeOperators implements CNNArchASTArchValueRangeCoCo {

    @Override
    public void check(ASTArchValueRange node) {
        if (node.isPresentParallel()){
            if (!node.isPresentParallel2()){
                differentOperatorError(node);
            }
        }
        else {
            if (node.isPresentParallel2()){
                differentOperatorError(node);
            }
        }
    }

    private void differentOperatorError(ASTArchValueRange node){
        Log.error("0" + ErrorCodes.DIFFERENT_RANGE_OPERATORS +
                        " the second layer operator ('->' or '|') in a range has to be identical to the first one."
                , node.get_SourcePositionStart());
    }

}
