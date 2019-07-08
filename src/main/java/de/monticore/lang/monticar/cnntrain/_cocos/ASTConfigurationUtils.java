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

import de.monticore.lang.monticar.cnntrain._ast.*;

import java.util.Optional;

class ASTConfigurationUtils {
    static boolean isReinforcementLearning(final ASTConfiguration configuration) {
        return configuration.getEntriesList().stream().anyMatch(e ->
                (e instanceof ASTLearningMethodEntry)
                        && ((ASTLearningMethodEntry)e).getValue().isPresentReinforcement());
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
                            && ((ASTEnvironmentEntry)e).getValue().getName().equals("ros_interface"));
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
}
