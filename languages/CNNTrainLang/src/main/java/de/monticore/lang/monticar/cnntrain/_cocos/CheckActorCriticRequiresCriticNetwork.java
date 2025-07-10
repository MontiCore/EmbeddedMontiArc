/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain._ast.ASTCriticNetworkEntry;
import de.monticore.lang.monticar.cnntrain._ast.ASTRLAlgorithmEntry;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import static de.monticore.lang.monticar.cnntrain._cocos.ASTConfigurationUtils.hasCriticEntry;
import static de.monticore.lang.monticar.cnntrain._cocos.ASTConfigurationUtils.isActorCriticAlgorithm;

public class CheckActorCriticRequiresCriticNetwork implements CNNTrainASTConfigurationCoCo {

    @Override
    public void check(ASTConfiguration node) {
        boolean isActorCritic = isActorCriticAlgorithm(node);
        boolean hasCriticEntry = hasCriticEntry(node);

        if (isActorCritic && !hasCriticEntry) {
            ASTRLAlgorithmEntry algorithmEntry = node.getEntriesList().stream()
                    .filter(e -> e instanceof ASTRLAlgorithmEntry)
                    .map(e -> (ASTRLAlgorithmEntry)e)
                    .findFirst()
                    .orElseThrow(() -> new IllegalStateException("ASTRLAlgorithmEntry entry must be available"));
            Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING + " DDPG learning algorithm requires critic" +
             " network entry", algorithmEntry.get_SourcePositionStart());
        }
    }
}