/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.*;
import static de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants.*;

import java.util.Optional;

class ASTConfigurationUtils {
    final static ParameterAlgorithmMapping parameterChecker = new ParameterAlgorithmMapping();

    static boolean isReinforcementLearning(final ASTConfiguration configuration) {
        return configuration.getEntriesList().stream().anyMatch(e ->
                (e instanceof ASTLearningMethodEntry)
                        && ((ASTLearningMethodEntry)e).getValue().isPresentReinforcement());
    }

    static boolean isGANLearning(final ASTConfiguration configuration) {
        return configuration.getEntriesList().stream().anyMatch(e ->
                (e instanceof ASTLearningMethodEntry)
                        && ((ASTLearningMethodEntry)e).getValue().isPresentGan());
    }

    static boolean hasEnvironment(final ASTConfiguration configuration) {
        return configuration.getEntriesList().stream().anyMatch(e -> e instanceof ASTEnvironmentEntry);
    }

    static boolean isDdpgAlgorithm(final ASTConfiguration configuration) {
        return isReinforcementLearning(configuration)
            && configuration.getEntriesList().stream().anyMatch(
            e -> (e instanceof ASTRLAlgorithmEntry) && ((ASTRLAlgorithmEntry)e).getValue().isPresentDdpg());
    }

    static boolean isTd3Algorithm(final ASTConfiguration configuration) {
        return isReinforcementLearning(configuration)
            && configuration.getEntriesList().stream().anyMatch(
            e -> (e instanceof ASTRLAlgorithmEntry) && ((ASTRLAlgorithmEntry)e).getValue().isPresentTdThree());
    }

    static boolean isDqnAlgorithm(final ASTConfiguration configuration) {
        return isReinforcementLearning(configuration)
            && !isDdpgAlgorithm(configuration)
            && !isTd3Algorithm(configuration);
    }

    static boolean hasEntry(final ASTConfiguration configuration, final Class<? extends ASTConfigEntry> entryClazz) {
        return configuration.getEntriesList().stream().anyMatch(entryClazz::isInstance);
    }

    static boolean hasStrategy(final ASTConfiguration configuration) {
        return configuration.getEntriesList().stream().anyMatch(e -> e instanceof ASTStrategyEntry);
    }

    static Optional<String> getStrategyMethod(final ASTConfiguration configuration) {
        return configuration.getEntriesList().stream()
            .filter(e -> e instanceof ASTStrategyEntry)
            .map(e -> (ASTStrategyEntry)e)
            .findFirst()
            .map(astStrategyEntry -> astStrategyEntry.getValue().getName());
    }

    static boolean hasRewardFunction(final ASTConfiguration node) {
        return node.getEntriesList().stream().anyMatch(e -> e instanceof ASTRewardFunctionEntry);
    }

    static boolean hasRosEnvironment(final ASTConfiguration node) {
        return ASTConfigurationUtils.hasEnvironment(node)
                && node.getEntriesList().stream()
                    .anyMatch(e -> (e instanceof ASTEnvironmentEntry)
                            && ((ASTEnvironmentEntry)e).getValue().getName().equals(ENVIRONMENT_ROS));
    }

    static boolean hasRewardTopic(final ASTConfiguration node) {
        if (ASTConfigurationUtils.isReinforcementLearning(node) && ASTConfigurationUtils.hasEnvironment(node)) {
            return node.getEntriesList().stream()
                .filter(ASTEnvironmentEntry.class::isInstance)
                .map(e -> (ASTEnvironmentEntry)e)
                .reduce((element, other) -> { throw new IllegalStateException("More than one entry");})
                .map(astEnvironmentEntry -> astEnvironmentEntry.getValue().getParamsList().stream()
                    .anyMatch(e -> e instanceof ASTRosEnvironmentRewardTopicEntry)).orElse(false);

        }
        return false;
    }

    static boolean isActorCriticAlgorithm(final ASTConfiguration node) {
       return isDdpgAlgorithm(node) || isTd3Algorithm(node);
    }

    static boolean hasCriticEntry(final ASTConfiguration node) {
        return node.getEntriesList().stream()
            .anyMatch(e -> ((e instanceof ASTCriticNetworkEntry)
                && !((ASTCriticNetworkEntry)e).getValue().getNameList().isEmpty()));
    }

    public static boolean isContinuousAlgorithm(final ASTConfiguration node) {
        return isDdpgAlgorithm(node) || isTd3Algorithm(node);
    }

    public static boolean hasRLEntry(ASTConfiguration node) {
        //return node.getEntriesList().stream()
        //    .anyMatch(e -> parameterChecker.isReinforcementLearningParameterOnly(e.getClass()));
        for (ASTConfigEntry e : node.getEntriesList()) {
            boolean b = parameterChecker.isReinforcementLearningParameterOnly(e.getClass());
            if (b) {
                return true;
            }
        }
        return false;
    }

    static boolean hasGeneratorLoss(final ASTConfiguration node) {
        return node.getEntriesList().stream().anyMatch(e -> e instanceof ASTGeneratorLossEntry);
    }

    static boolean hasGeneratorTargetName(final ASTConfiguration node) {
        return node.getEntriesList().stream().anyMatch(e -> e instanceof ASTGeneratorTargetNameEntry);
    }

    static boolean hasNoiseName(final ASTConfiguration node) {
        return node.getEntriesList().stream().anyMatch(e -> e instanceof ASTNoiseInputEntry);
    }

    static boolean hasNoiseDistribution(final ASTConfiguration node) {
        return node.getEntriesList().stream().anyMatch(e -> e instanceof ASTNoiseDistributionEntry);
    }

    static boolean hasConstraintDistribution(final ASTConfiguration node) {
        return node.getEntriesList().stream().anyMatch(e -> e instanceof ASTConstraintDistributionEntry);
    }

    static boolean hasConstraintLosses(final ASTConfiguration node) {
        return node.getEntriesList().stream().anyMatch(e -> e instanceof ASTConstraintLossEntry);
    }

    static boolean hasQNetwork(final ASTConfiguration node) {
        return node.getEntriesList().stream().anyMatch(e -> e instanceof ASTQNetworkEntry);
    }
}
