/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import static de.monticore.lang.monticar.cnntrain._cocos.ASTConfigurationUtils.*;

public class CheckNoiseInputMissing implements CNNTrainASTConfigurationCoCo {
    @Override
    public void check(final ASTConfiguration node) {
        if (isGANLearning(node)) {
            if (!hasNoiseName(node)) {
                Log.warn(ErrorCodes.MISSING_PARAMETER_VALUE_CODE + " No noise input specified. Are you sure this is correct?");
            }
        }
    }
}
