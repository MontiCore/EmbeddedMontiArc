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

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnntrain._ast.*;

import java.util.List;

class ParameterAlgorithmMapping {
    private static final List<Class> GENERAL_PARAMETERS = Lists.newArrayList(
        ASTTrainContextEntry.class,
        ASTOptimizerEntry.class,
        ASTLearningRateEntry.class,
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
        ASTBeta2Entry.class
    );

    private static final List<Class> EXCLUSIVE_SUPERVISED_PARAMETERS = Lists.newArrayList(
        ASTBatchSizeEntry.class,
        ASTLoadCheckpointEntry.class,
        ASTEvalMetricEntry.class,
        ASTNormalizeEntry.class,
        ASTNumEpochEntry.class,
        ASTLossEntry.class,
        ASTSparseLabelEntry.class,
        ASTFromLogitsEntry.class,
        ASTMarginEntry.class,
        ASTLabelFormatEntry.class,
        ASTRhoEntry.class
    );

    private static final List<Class> GENERAL_REINFORCEMENT_PARAMETERS = Lists.newArrayList(
        ASTRLAlgorithmEntry.class,
        ASTRewardFunctionEntry.class,
        ASTDiscountFactorEntry.class,
        ASTNumMaxStepsEntry.class,
        ASTTargetScoreEntry.class,
        ASTTrainingIntervalEntry.class,
        ASTAgentNameEntry.class,
        ASTGymEnvironmentNameEntry.class,
        ASTEnvironmentEntry.class,
        ASTReplayMemoryEntry.class,
        ASTMemorySizeEntry.class,
        ASTSampleSizeEntry.class,
        ASTStrategyEntry.class,
        ASTGreedyEpsilonEntry.class,
        ASTMinEpsilonEntry.class,
        ASTEpsilonDecayEntry.class,
        ASTEpsilonDecayMethodEntry.class,
        ASTNumEpisodesEntry.class,
        ASTRosEnvironmentActionTopicEntry.class,
        ASTRosEnvironmentStateTopicEntry.class,
        ASTRosEnvironmentResetTopicEntry.class,
        ASTRosEnvironmentTerminalStateTopicEntry.class,
        ASTRosEnvironmentRewardTopicEntry.class,
        ASTStartTrainingAtEntry.class,
        ASTEvaluationSamplesEntry.class,
        ASTEpsilonDecayStartEntry.class,
        ASTSnapshotIntervalEntry.class
    );

    private static final List<Class> EXCLUSIVE_DQN_PARAMETERS = Lists.newArrayList(
        ASTUseFixTargetNetworkEntry.class,
        ASTTargetNetworkUpdateIntervalEntry.class,
        ASTUseDoubleDQNEntry.class,
        ASTLossEntry.class
    );

    private static final List<Class> EXCLUSIVE_DDPG_PARAMETERS = Lists.newArrayList(
        ASTCriticNetworkEntry.class,
        ASTSoftTargetUpdateRateEntry.class,
        ASTCriticOptimizerEntry.class,
        ASTStrategyOUMu.class,
        ASTStrategyOUTheta.class,
        ASTStrategyOUSigma.class
    );

    ParameterAlgorithmMapping() {

    }

    boolean isReinforcementLearningParameter(Class<? extends ASTEntry> entryClazz) {
        return GENERAL_PARAMETERS.contains(entryClazz)
            || GENERAL_REINFORCEMENT_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_DQN_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_DDPG_PARAMETERS.contains(entryClazz);
    }

    boolean isSupervisedLearningParameter(Class<? extends ASTEntry> entryClazz) {
        return GENERAL_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_SUPERVISED_PARAMETERS.contains(entryClazz);
    }

    boolean isDqnParameter(Class<? extends ASTEntry> entryClazz) {
        return GENERAL_PARAMETERS.contains(entryClazz)
            || GENERAL_REINFORCEMENT_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_DQN_PARAMETERS.contains(entryClazz);
    }

    boolean isDdpgParameter(Class<? extends ASTEntry> entryClazz) {
        return GENERAL_PARAMETERS.contains(entryClazz)
            || GENERAL_REINFORCEMENT_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_DDPG_PARAMETERS.contains(entryClazz);
    }

    List<Class> getAllReinforcementParameters() {
        return ImmutableList.<Class> builder()
            .addAll(GENERAL_PARAMETERS)
            .addAll(GENERAL_REINFORCEMENT_PARAMETERS)
            .addAll(EXCLUSIVE_DQN_PARAMETERS)
            .addAll(EXCLUSIVE_DDPG_PARAMETERS)
            .build();
    }

    List<Class> getAllSupervisedParameters() {
        return ImmutableList.<Class> builder()
            .addAll(GENERAL_PARAMETERS)
            .addAll(EXCLUSIVE_SUPERVISED_PARAMETERS)
            .build();
    }
}