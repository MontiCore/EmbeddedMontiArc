/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnntrain._cocos;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.lang.monticar.cnntrain._symboltable.LearningMethod;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 *
 */
public class CheckLearningParameterCombination implements CNNTrainASTEntryCoCo {
    private final static List<Class> ALLOWED_SUPERVISED_LEARNING = Lists.newArrayList(
            ASTTrainContextEntry.class,
            ASTBatchSizeEntry.class,
            ASTOptimizerEntry.class,
            ASTLearningRateEntry.class,
            ASTLoadCheckpointEntry.class,
            ASTEvalMetricEntry.class,
            ASTLossEntry.class,
            ASTNormalizeEntry.class,
            ASTMinimumLearningRateEntry.class,
            ASTLRDecayEntry.class,
            ASTWeightDecayEntry.class,
            ASTLRPolicyEntry.class,
            ASTStepSizeEntry.class,
            ASTRescaleGradEntry.class,
            ASTClipGradEntry.class,
            ASTGamma1Entry.class,
            ASTGamma2Entry.class,
            ASTEpsilonEntry.class,
            ASTCenteredEntry.class,
            ASTClipWeightsEntry.class,
            ASTBeta1Entry.class,
            ASTBeta2Entry.class,
            ASTNumEpochEntry.class
    );
    private final static List<Class> ALLOWED_REINFORCEMENT_LEARNING = Lists.newArrayList(
            ASTTrainContextEntry.class,
            ASTRLAlgorithmEntry.class,
            ASTCriticNetworkEntry.class,
            ASTOptimizerEntry.class,
            ASTRewardFunctionEntry.class,
            ASTMinimumLearningRateEntry.class,
            ASTLRDecayEntry.class,
            ASTWeightDecayEntry.class,
            ASTLRPolicyEntry.class,
            ASTGamma1Entry.class,
            ASTGamma2Entry.class,
            ASTEpsilonEntry.class,
            ASTClipGradEntry.class,
            ASTRescaleGradEntry.class,
            ASTStepSizeEntry.class,
            ASTCenteredEntry.class,
            ASTClipWeightsEntry.class,
            ASTLearningRateEntry.class,
            ASTDiscountFactorEntry.class,
            ASTNumMaxStepsEntry.class,
            ASTTargetScoreEntry.class,
            ASTTrainingIntervalEntry.class,
            ASTUseFixTargetNetworkEntry.class,
            ASTTargetNetworkUpdateIntervalEntry.class,
            ASTSnapshotIntervalEntry.class,
            ASTAgentNameEntry.class,
            ASTGymEnvironmentNameEntry.class,
            ASTEnvironmentEntry.class,
            ASTUseDoubleDQNEntry.class,
            ASTLossEntry.class,
            ASTReplayMemoryEntry.class,
            ASTMemorySizeEntry.class,
            ASTSampleSizeEntry.class,
            ASTActionSelectionEntry.class,
            ASTGreedyEpsilonEntry.class,
            ASTMinEpsilonEntry.class,
            ASTEpsilonDecayEntry.class,
            ASTEpsilonDecayMethodEntry.class,
            ASTNumEpisodesEntry.class,
            ASTRosEnvironmentActionTopicEntry.class,
            ASTRosEnvironmentStateTopicEntry.class,
            ASTRosEnvironmentMetaTopicEntry.class,
            ASTRosEnvironmentResetTopicEntry.class,
            ASTRosEnvironmentTerminalStateTopicEntry.class,
            ASTRosEnvironmentGreetingTopicEntry.class
    );

    private Set<ASTEntry> allEntries;

    private Boolean learningMethodKnown;
    private LearningMethod learningMethod;

    public CheckLearningParameterCombination() {
        this.allEntries = new HashSet<>();
        this.learningMethodKnown = false;
    }

    private Boolean isLearningMethodKnown() {
        return this.learningMethodKnown;
    }

    @Override
    public void check(ASTEntry node) {
        if (node instanceof ASTLearningMethodEntry) {
            evaluateLearningMethodEntry(node);
        } else {
            evaluateEntry(node);
        }
    }

    private void evaluateEntry(ASTEntry node) {
        allEntries.add(node);
        final Boolean supervisedLearningParameter = ALLOWED_SUPERVISED_LEARNING.contains(node.getClass());
        final Boolean reinforcementLearningParameter = ALLOWED_REINFORCEMENT_LEARNING.contains(node.getClass());

        assert (supervisedLearningParameter || reinforcementLearningParameter) :
                "Parameter " + node.getName() + " is not checkable, because it is unknown to Condition";
        if (supervisedLearningParameter && reinforcementLearningParameter) {
            return;
        } else if (supervisedLearningParameter && !reinforcementLearningParameter) {
            setLearningMethodOrLogErrorIfActualLearningMethodIsNotSupervised(node);
        } else if (!supervisedLearningParameter && reinforcementLearningParameter) {
            setLearningMethodOrLogErrorIfActualLearningMethodIsNotReinforcement(node);
        }
    }

    private void setLearningMethodOrLogErrorIfActualLearningMethodIsNotReinforcement(ASTEntry node) {
        if (isLearningMethodKnown()) {
            if (!learningMethod.equals(LearningMethod.REINFORCEMENT)) {
                Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER + " Parameter "
                                + node.getName() + " is not supported for " + this.learningMethod + " learning.",
                        node.get_SourcePositionStart());
            }
        } else {
            setLearningMethodToReinforcement();
        }
    }

    private void setLearningMethodOrLogErrorIfActualLearningMethodIsNotSupervised(ASTEntry node) {
        if (isLearningMethodKnown()) {
            if (!learningMethod.equals(LearningMethod.SUPERVISED)) {
                Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER + " Parameter "
                                + node.getName() + " is not supported for " + this.learningMethod + " learning.",
                        node.get_SourcePositionStart());
            }
        } else {
            setLearningMethodToSupervised();
        }
    }

    private void evaluateLearningMethodEntry(ASTEntry node) {
        ASTLearningMethodValue learningMethodValue = (ASTLearningMethodValue)node.getValue();
        LearningMethod evaluatedLearningMethod = learningMethodValue.isPresentReinforcement()
                ? LearningMethod.REINFORCEMENT : LearningMethod.SUPERVISED;

        if (isLearningMethodKnown()) {
            logErrorIfEvaluatedLearningMethoNotEqualToActual(node, evaluatedLearningMethod);
        } else {
            setLearningMethod(evaluatedLearningMethod);
        }
    }

    private void logErrorIfEvaluatedLearningMethoNotEqualToActual(ASTEntry node, LearningMethod evaluatedLearningMethod) {
        if (!evaluatedLearningMethod.equals(this.learningMethod)) {
            String wrongParameter = findWrongParameter(evaluatedLearningMethod);
            Log.error("0" + ErrorCodes.UNSUPPORTED_PARAMETER
                    + " Parameter " + wrongParameter + " for " + evaluatedLearningMethod + ".",
                    node.get_SourcePositionStart());
        }
    }

    private String findWrongParameter(LearningMethod learningMethod) {
        List<Class> allowedParameters = getAllowedParametersByLearningMethod(learningMethod);
        for (ASTEntry entry : this.allEntries) {
            if (!allowedParameters.contains(entry.getClass())) {
                return entry.getName();
            }
        }
        return null;
    }

    private List<Class> getAllowedParametersByLearningMethod(final LearningMethod learningMethod) {
        if (learningMethod.equals(LearningMethod.REINFORCEMENT)) {
            return ALLOWED_REINFORCEMENT_LEARNING;
        }
        return ALLOWED_SUPERVISED_LEARNING;
    }

    private void setLearningMethod(final LearningMethod learningMethod) {
        if (learningMethod.equals(LearningMethod.REINFORCEMENT)) {
            setLearningMethodToReinforcement();
        } else {
            setLearningMethodToSupervised();
        }
    }

    private void setLearningMethodToSupervised() {
        this.learningMethod = LearningMethod.SUPERVISED;
        this.learningMethodKnown = true;
    }

    private void setLearningMethodToReinforcement() {
        this.learningMethod = LearningMethod.REINFORCEMENT;
        this.learningMethodKnown = true;
    }
}
