/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Sets;
import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

public class CheckEntryRepetition implements CNNTrainASTEntryCoCo {
    private final static Set<Class<? extends ASTEntry>> REPEATABLE_ENTRIES = ImmutableSet
        .<Class<? extends ASTEntry>>builder()
            .add(ASTOptimizerParamEntry.class)
            .add(ASTNoiseDistributionParamEntry.class)
            .build();



    private Set<String> entryNameSet = new HashSet<>();

    @Override
    public void check(ASTEntry node) {
        if (!isRepeatable(node)) {
            String parameterPrefix = "";

            if (node instanceof ASTGreedyEpsilonEntry) {
                parameterPrefix = "greedy_";
            }
            if (entryNameSet.contains(parameterPrefix + node.getName())) {
                Log.error("0" + ErrorCodes.ENTRY_REPETITION_CODE + " The parameter '" + node.getName() + "' has multiple values. " +
                                "Multiple assignments of the same parameter are not allowed",
                        node.get_SourcePositionStart());
            } else {
                entryNameSet.add(parameterPrefix + node.getName());
            }
        }
    }

    private boolean isRepeatable(final ASTEntry node) {
        return REPEATABLE_ENTRIES.stream().anyMatch(i -> i.isInstance(node));
    }

}
