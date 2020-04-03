/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTEntry;
import de.monticore.lang.monticar.cnntrain._ast.ASTLearningMethodEntry;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.LearningMethod;
import de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;
import sun.security.krb5.internal.ccache.CredentialsCache;

public class CheckGANConfigurationDependencies implements CNNTrainConfigurationSymbolCoCo{

    public CheckGANConfigurationDependencies() { }

    @Override
    public void check(ConfigurationSymbol configurationSymbol) {

        if(configurationSymbol.getLearningMethod() == LearningMethod.GAN) {

            if (configurationSymbol.getEntry(ConfigEntryNameConstants.GENERATOR_LOSS) != null)
                if (configurationSymbol.getEntry(ConfigEntryNameConstants.GENERATOR_TARGET_NAME) == null)
                    Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING +
                            " Generator loss specified but conditional input is missing");

            if (configurationSymbol.getEntry(ConfigEntryNameConstants.GENERATOR_TARGET_NAME) != null)
                if (configurationSymbol.getEntry(ConfigEntryNameConstants.GENERATOR_LOSS) == null)
                    Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING +
                            " Conditional input specified but generator loss is missing");

            if (configurationSymbol.getEntry(ConfigEntryNameConstants.LOSS) != null)
                Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER +
                        " Loss parameter not valid for GAN learning");

            if (configurationSymbol.getEntry(ConfigEntryNameConstants.NOISE_INPUT) != null)
                if (configurationSymbol.getEntry(ConfigEntryNameConstants.NOISE_DISTRIBUTION) == null)
                    Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING +
                            " Noise input specified but noise distribution parameter is missing");

            if (configurationSymbol.getEntry(ConfigEntryNameConstants.CONSTRAINT_DISTRIBUTION) != null)
                if (configurationSymbol.getEntry(ConfigEntryNameConstants.QNETWORK_NAME) == null)
                    Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING +
                            " Constraint distributions are given but q-network is missing");

            if (configurationSymbol.getEntry(ConfigEntryNameConstants.CONSTRAINT_LOSS) != null)
                if (configurationSymbol.getEntry(ConfigEntryNameConstants.QNETWORK_NAME) == null)
                    Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING +
                            " Constraint losses are given but q-network is missing");

            if (configurationSymbol.getEntry(ConfigEntryNameConstants.NOISE_INPUT) == null)
                Log.warn(" No noise input specified. Are you sure this is correct?");
        }
    }
}
