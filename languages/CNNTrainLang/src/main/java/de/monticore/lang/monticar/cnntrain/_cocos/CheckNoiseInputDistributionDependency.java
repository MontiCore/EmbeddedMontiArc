/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import static de.monticore.lang.monticar.cnntrain._cocos.ASTConfigurationUtils.*;

public class CheckNoiseInputDistributionDependency implements CNNTrainASTConfigurationCoCo {
    @Override
    public void check(final ASTConfiguration node) {
        if (isGANLearning(node) && hasNoiseName(node)) {
            if (!hasNoiseDistribution(node)) {
                Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING +
                        " Noise input specified but noise distribution parameter is missing");
            }
        }
    }
}
