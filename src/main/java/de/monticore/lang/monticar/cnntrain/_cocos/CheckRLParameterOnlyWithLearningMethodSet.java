/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckRLParameterOnlyWithLearningMethodSet implements CNNTrainASTConfigurationCoCo {
    @Override
    public void check(ASTConfiguration node) {
        if (!ASTConfigurationUtils.isReinforcementLearning(node) && ASTConfigurationUtils.hasRLEntry(node)) {
            Log.error(ErrorCodes.REQUIRED_PARAMETER_MISSING + " Reinforcement parameter used although learning " +
             "method not set", node.get_SourcePositionStart());
        }
    }
}
