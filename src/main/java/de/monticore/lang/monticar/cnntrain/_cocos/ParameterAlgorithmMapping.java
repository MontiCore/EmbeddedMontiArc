/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnntrain._ast.*;

import java.util.List;

class ParameterAlgorithmMapping {
    private static final List<Class> GENERAL_PARAMETERS = Lists.newArrayList(
        ASTTrainContextEntry.class,
        ASTInitializerEntry.class,
        ASTInitializerNormalSigma.class,
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
        ASTCheckpointPeriodEntry.class,
        ASTLoadPretrainedEntry.class,
        ASTLogPeriodEntry.class,
        ASTEvalMetricEntry.class,
        ASTEvalTrainEntry.class,
        ASTExcludeBleuEntry.class,
        ASTAxisAccIgnoreLabelEntry.class,
        ASTIgnoreLabelAccIgnoreLabelEntry.class,
        ASTNormalizeEntry.class,
        ASTNumEpochEntry.class,
        ASTLossEntry.class,
		ASTLossWeightsEntry.class,
        ASTSparseLabelEntry.class,
        ASTLossAxisEntry.class,
        ASTBatchAxisEntry.class,
        ASTFromLogitsEntry.class,
        ASTIgnoreIndicesEntry.class,
        ASTIgnoreLabelEntry.class,
        ASTMarginEntry.class,
        ASTLabelFormatEntry.class,
        ASTRhoEntry.class,
        ASTPreprocessingEntry.class,
        ASTUseTeacherForcing.class,
        ASTSaveAttentionImage.class
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
        ASTEpsilonDecayPerStepEntry.class,
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
        ASTCriticInitializerEntry.class,
        ASTCriticOptimizerEntry.class,
        ASTStrategyOUMu.class,
        ASTStrategyOUTheta.class,
        ASTStrategyOUSigma.class,
        ASTStrategyGaussianNoiseVarianceEntry.class
    );

    private static final List<Class> EXCLUSIVE_TD3_PARAMETERS = Lists.newArrayList(
        ASTCriticNetworkEntry.class,
        ASTSoftTargetUpdateRateEntry.class,
        ASTCriticInitializerEntry.class,
        ASTCriticOptimizerEntry.class,
        ASTStrategyOUMu.class,
        ASTStrategyOUTheta.class,
        ASTStrategyOUSigma.class,
        ASTPolicyNoiseEntry.class,
        ASTNoiseClipEntry.class,
        ASTPolicyDelayEntry.class,
        ASTStrategyGaussianNoiseVarianceEntry.class
    );

    private static final List<Class> GENERAL_GAN_PARAMETERS = Lists.newArrayList(
            ASTDiscriminatorNetworkEntry.class,
            ASTPreprocessingEntry.class,
            ASTQNetworkEntry.class,
            ASTNoiseDistributionEntry.class,
            ASTConstraintDistributionEntry.class,
            ASTConstraintLossEntry.class,
            ASTDiscriminatorOptimizerEntry.class,
            ASTKValueEntry.class,
            ASTGeneratorLossEntry.class,
            ASTGeneratorTargetNameEntry.class,
            ASTNoiseInputEntry.class,
            ASTGeneratorLossWeightEntry.class,
            ASTDiscriminatorLossWeightEntry.class,
            ASTPrintImagesEntry.class,
            ASTMeanValueEntry.class,
            ASTSpreadValueEntry.class,
            ASTBatchSizeEntry.class,
            ASTLoadCheckpointEntry.class,
            ASTCheckpointPeriodEntry.class,
            ASTLoadPretrainedEntry.class,
            ASTLogPeriodEntry.class,
            ASTNormalizeEntry.class,
            ASTNumEpochEntry.class,
            ASTLossWeightsEntry.class,
            ASTSparseLabelEntry.class,
            ASTLossAxisEntry.class,
            ASTBatchAxisEntry.class,
            ASTFromLogitsEntry.class,
            ASTIgnoreIndicesEntry.class,
            ASTIgnoreLabelEntry.class,
            ASTMarginEntry.class,
            ASTLabelFormatEntry.class,
            ASTRhoEntry.class
    );

    ParameterAlgorithmMapping() {

    }

    boolean isReinforcementLearningParameter(Class<? extends ASTEntry> entryClazz) {
        return GENERAL_PARAMETERS.contains(entryClazz)
            || GENERAL_REINFORCEMENT_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_DQN_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_DDPG_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_TD3_PARAMETERS.contains(entryClazz);
    }

    boolean isReinforcementLearningParameterOnly(Class<? extends ASTEntry> entryClazz) {
        return (GENERAL_REINFORCEMENT_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_DQN_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_DDPG_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_TD3_PARAMETERS.contains(entryClazz))
            && !GENERAL_PARAMETERS.contains(entryClazz)
            && !EXCLUSIVE_SUPERVISED_PARAMETERS.contains(entryClazz);
    }

    boolean isSupervisedLearningParameter(Class<? extends ASTEntry> entryClazz) {
        return GENERAL_PARAMETERS.contains(entryClazz)
                || EXCLUSIVE_SUPERVISED_PARAMETERS.contains(entryClazz);
    }

    boolean isGANLearningParameter(Class<? extends ASTEntry> entryClazz) {
        return GENERAL_PARAMETERS.contains(entryClazz)
                || GENERAL_GAN_PARAMETERS.contains(entryClazz);

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

    boolean isTd3Parameter(Class<? extends ASTEntry> entryClazz) {
        return GENERAL_PARAMETERS.contains(entryClazz)
            || GENERAL_REINFORCEMENT_PARAMETERS.contains(entryClazz)
            || EXCLUSIVE_TD3_PARAMETERS.contains(entryClazz);
    }

    List<Class> getAllGANParameters() {
        return ImmutableList.<Class> builder()
                .addAll(GENERAL_PARAMETERS)
                .addAll(GENERAL_GAN_PARAMETERS)
                .build();
    }

    List<Class> getAllReinforcementParameters() {
        return ImmutableList.<Class> builder()
            .addAll(GENERAL_PARAMETERS)
            .addAll(GENERAL_REINFORCEMENT_PARAMETERS)
            .addAll(EXCLUSIVE_DQN_PARAMETERS)
            .addAll(EXCLUSIVE_DDPG_PARAMETERS)
            .addAll(EXCLUSIVE_TD3_PARAMETERS)
            .build();
    }

    List<Class> getAllSupervisedParameters() {
        return ImmutableList.<Class> builder()
            .addAll(GENERAL_PARAMETERS)
            .addAll(EXCLUSIVE_SUPERVISED_PARAMETERS)
            .build();
    }
}
