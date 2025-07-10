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

public class CheckConstraintLossesQNetworkDependency implements CNNTrainASTConfigurationCoCo {
    @Override
    public void check(final ASTConfiguration node) {
        if (isGANLearning(node) && hasConstraintLosses(node)) {
            if (!hasQNetwork(node)) {
                Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING +
                        " Constraint losses are given but q-network is missing");
            }
        }
    }
}
