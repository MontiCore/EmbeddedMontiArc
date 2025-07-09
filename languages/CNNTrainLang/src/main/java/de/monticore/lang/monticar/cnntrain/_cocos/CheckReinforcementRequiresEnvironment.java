/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain._ast.ASTEnvironmentEntry;
import de.monticore.lang.monticar.cnntrain._ast.ASTLearningMethodEntry;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.LearningMethod;
import de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

/**
 *
 */
public class CheckReinforcementRequiresEnvironment implements CNNTrainASTConfigurationCoCo {
    @Override
    public void check(ASTConfiguration node) {
        boolean isReinforcementLearning = ASTConfigurationUtils.isReinforcementLearning(node);
        boolean hasEnvironment = ASTConfigurationUtils.hasEnvironment(node);

        if (isReinforcementLearning && !hasEnvironment) {
            Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING + " The required parameter "
                    + ConfigEntryNameConstants.ENVIRONMENT + " is missing");
        }
    }
}
