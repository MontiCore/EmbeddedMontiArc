/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.EntrySymbol;
import de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Map;

/**
 *
 */
public class CheckFixTargetNetworkRequiresInterval implements CNNTrainASTConfigurationCoCo {

    @Override
    public void check(ASTConfiguration node) {
        boolean useFixTargetNetwork = node.getEntriesList().stream()
                .anyMatch(e -> e instanceof ASTUseFixTargetNetworkEntry
                    && ((ASTUseFixTargetNetworkEntry)e).getValue().isPresentTRUE());
        boolean hasTargetNetworkUpdateInterval = node.getEntriesList().stream()
                .anyMatch(e -> (e instanceof ASTTargetNetworkUpdateIntervalEntry));

        if (useFixTargetNetwork && !hasTargetNetworkUpdateInterval) {
            ASTUseFixTargetNetworkEntry useFixTargetNetworkEntry = node.getEntriesList().stream()
                    .filter(e -> e instanceof ASTUseFixTargetNetworkEntry)
                    .map(e -> (ASTUseFixTargetNetworkEntry)e)
                    .findFirst()
                    .orElseThrow(() -> new IllegalStateException("ASTUseFixTargetNetwork entry must be available"));
            Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING + " Parameter " + ConfigEntryNameConstants.USE_FIX_TARGET_NETWORK
                    + " requires parameter " + ConfigEntryNameConstants.TARGET_NETWORK_UPDATE_INTERVAL,
                    useFixTargetNetworkEntry.get_SourcePositionStart());
        } else if (!useFixTargetNetwork && hasTargetNetworkUpdateInterval) {
            ASTTargetNetworkUpdateIntervalEntry targetNetworkUpdateIntervalEntry = node.getEntriesList().stream()
                    .filter(e -> e instanceof ASTTargetNetworkUpdateIntervalEntry)
                    .map(e -> (ASTTargetNetworkUpdateIntervalEntry)e)
                    .findFirst()
                    .orElseThrow(
                            () -> new IllegalStateException("ASTTargetNetworkUpdateInterval entry must be available"));
            Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING + " Parameter "
                            + targetNetworkUpdateIntervalEntry.getName() + " requires that parameter "
                            + ConfigEntryNameConstants.USE_FIX_TARGET_NETWORK + " to be true.",
                    targetNetworkUpdateIntervalEntry.get_SourcePositionStart());
        }
    }
}
