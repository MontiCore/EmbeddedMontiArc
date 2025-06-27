/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import de.monticore.lang.monticar.cnnarch.generator.annotations.ArchitectureAdapter;
import de.monticore.lang.monticar.cnnarch.generator.annotations.Range;
import de.monticore.lang.monticar.cnnarch.generator.training.RlAlgorithm;
import de.monticore.lang.monticar.cnnarch.generator.training.NetworkType;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import static de.monticore.lang.monticar.cnnarch.generator.training.TrainingParameterConstants.*;

public abstract class ConfigurationData {

    private TrainingConfiguration trainingConfiguration;
    protected TrainingComponentsContainer trainingComponentsContainer;
    private String instanceName;
    private TrainParamSupportChecker trainParamSupportChecker;

    public ConfigurationData(TrainingConfiguration trainingConfiguration, TrainingComponentsContainer trainingComponentsContainer,
                             String instanceName, TrainParamSupportChecker trainParamSupportChecker) {
        this.trainingConfiguration = trainingConfiguration;
        this.trainingComponentsContainer = trainingComponentsContainer;
        this.instanceName = instanceName;
        this.trainParamSupportChecker = trainParamSupportChecker;
    }

    public Boolean isSupervisedLearning() {
        return trainingConfiguration.isSupervisedLearning();
    }

    public boolean isGanLearning() {
        return trainingConfiguration.isGanLearning();
    }

    public Boolean isVaeLearning() {
        return trainingConfiguration.isVaeLearning();
    }

    public Boolean isReinforcementLearning() {
        return trainingConfiguration.isReinforcementLearning();
    }

    public Integer getBatchSize() {
        Optional<Integer> batchSizeOpt = trainingConfiguration.getBatchSize();
        return batchSizeOpt.orElse(null);
    }

    public Integer getNumEpoch() {
        Optional<Integer> numEpochOpt = trainingConfiguration.getNumEpoch();
        return numEpochOpt.orElse(null);
    }

    public Boolean getLoadCheckpoint() {
        Optional<Boolean> loadCheckpointOpt = trainingConfiguration.getLoadCheckpoint();
        return loadCheckpointOpt.orElse(null);
    }

    public Integer getCheckpointPeriod() {
        Optional<Integer> checkpointPeriodOpt = trainingConfiguration.getCheckpointPeriod();
        return checkpointPeriodOpt.orElse(null);
    }

    public Integer getLogPeriod() {
        Optional<Integer> logPeriodOpt = trainingConfiguration.getLogPeriod();
        return logPeriodOpt.orElse(null);
    }

    public Boolean getLoadPretrained() {
        Optional<Boolean> loadPretrainedOpt = trainingConfiguration.getLoadPretrained();
        return loadPretrainedOpt.orElse(null);
    }

    public Double getKlLossWeight() {
        Optional<Double> klLossWeightOpt = trainingConfiguration.getKlLossWeight();
        return klLossWeightOpt.orElse(null);
    }

    public String getReconLossName() {
        Optional<String> reconLossNameOpt = trainingConfiguration.getReconLossName();
        return reconLossNameOpt.orElse(null);
    }

    // COMPARE WITH CNNTRAIN IMPLEMENTATION IN GluonConfigurationData
    public Boolean getPreprocessor() {
        Optional<String> preprocessorOpt = trainingConfiguration.getPreprocessor();
        return preprocessorOpt.isPresent();
    }

    public Boolean getNormalize() {
        Optional<Boolean> normalizeOpt = trainingConfiguration.getNormalize();
        return normalizeOpt.orElse(null);
    }

    public Boolean getCleaning() {
        return trainingConfiguration.hasCleaning() ? true : null;
    }

    public String getCleaningName() {
        Optional<String> cleaningNameOpt = trainingConfiguration.getCleaningName();
        return cleaningNameOpt.orElse(null);
    }

    public Map<String, Object> getCleaningParameters() {
        if (!getCleaning()) {
            return null;
        }
        Map<String, Object> cleaningParameters = trainingConfiguration.getCleaningParameters();
        return formatParameters(cleaningParameters);
    }

    public Boolean getDataImbalance() {
        return trainingConfiguration.hasDataImbalance() ? true : null;
    }

    public String getDataImbalanceName() {
        Optional<String> dataImbalanceNameOpt = trainingConfiguration.getDataImbalanceName();
        return dataImbalanceNameOpt.orElse(null);
    }

    public Map<String, Object> getDataImbalanceParameters() {
        if (!getDataImbalance()) {
            return null;
        }
        Map<String, Object> dataImbalanceParameters = trainingConfiguration.getDataImbalanceParameters();
        return formatParameters(dataImbalanceParameters);
    }

    public Boolean getDataSplitting() {
        return trainingConfiguration.hasDataSplitting() ? true : null;
    }

    public String getDataSplittingName() {
        Optional<String> dataSplittingNameOpt = trainingConfiguration.getDataSplittingName();
        return dataSplittingNameOpt.orElse(null);
    }

    public Map<String, Object> getDataSplittingParameters() {
        if (!getDataSplitting()) {
            return null;
        }
        Map<String, Object> dataSplittingParameters = trainingConfiguration.getDataSplittingParameters();
        return formatParameters(dataSplittingParameters);
    }

    public Boolean getOnnxExport() {
        Optional<Boolean> onnxExport = trainingConfiguration.getOnnxExport();
        return onnxExport.orElse(null);
    }

    public Boolean getMultiGraph() {
        Optional<Boolean> multiGraphOpt = trainingConfiguration.getMultiGraph();
        return multiGraphOpt.orElse(null);
    }

    public List<Integer> getTrainMask() {
        Optional<List<Integer>> trainMaskOpt = trainingConfiguration.getTrainMask();
        return trainMaskOpt.orElse(null);
    }

    public List<Integer> getTestMask() {
        Optional<List<Integer>> testMaskOpt = trainingConfiguration.getTestMask();
        return testMaskOpt.orElse(null);
    }

    public Boolean getShuffleData() {
        Optional<Boolean> shuffleDataOpt = trainingConfiguration.getShuffleData();
        return shuffleDataOpt.orElse(null);
    }

    public Double getClipGlobalGradNorm() {
        Optional<Double> clipGlobalGradNormOpt = trainingConfiguration.getClipGlobalGradNorm();
        return clipGlobalGradNormOpt.orElse(null);
    }

    public Boolean getUseTeacherForcing() {
        Optional<Boolean> useTeacherForcingOpt = trainingConfiguration.getUseTeacherForcing();
        return useTeacherForcingOpt.orElse(null);
    }

    public Boolean getSaveAttentionImage() {
        Optional<Boolean> saveAttentionImageOpt = trainingConfiguration.getSaveAttentionImage();
        return saveAttentionImageOpt.orElse(null);
    }

    public Boolean getEvalTrain() {
        Optional<Boolean> evalTrainOpt = trainingConfiguration.getEvalTrain();
        return evalTrainOpt.orElse(null);
    }

    public String getContext() { // TODO enum?
        Optional<String> context = trainingConfiguration.getContext();
        return context.orElse(null);
    }

    public Boolean getOptimizer() {
        if (!trainingConfiguration.hasOptimizer()) return null;

        Collection<String> unsupportedOptimizers = trainParamSupportChecker.getUnsupportedOptimizers();
        if (unsupportedOptimizers.contains(trainingConfiguration.getOptimizerName().get())) {
            return null;
        }
        return true;
    }

    public String getOptimizerName() {
        Optional<String> optimizerNameOpt = trainingConfiguration.getOptimizerName();
        return optimizerNameOpt.orElse(null);
    }

    public Map<String, Object> getOptimizerParameters() {
        if (!getOptimizer()) {
            return null;
        }

        Map<String, Object> optimizerParameters = trainingConfiguration.getOptimizerParameters();
        if (optimizerParameters.isEmpty()) {
            return optimizerParameters;
        }

        Map<String, Object> optimizerParameterMapFiltered = Maps.newHashMap();
        Collection<String> unsupportedOptimizerParameters = trainParamSupportChecker.getUnsupportedOptimizerParameters();
        for (Map.Entry<String, Object> parameter : optimizerParameters.entrySet()) {
            if (unsupportedOptimizerParameters.contains(parameter.getKey())) {
                continue;
            }
            optimizerParameterMapFiltered.put(parameter.getKey(), parameter.getValue());
        }
        return formatParameters(optimizerParameterMapFiltered);
    }

    public Boolean getActorOptimizer() {
        return trainingConfiguration.hasActorOptimizer() ? true : null;
    }

    public String getActorOptimizerName() {
        Optional<String> actorOptimizerNameOpt = trainingConfiguration.getActorOptimizerName();
        return actorOptimizerNameOpt.orElse(null);
    }

    public Map<String, Object> getActorOptimizerParameters() {
        if (!getActorOptimizer()) {
            return null;
        }
        Map<String, Object> actorOptimizerParameters = trainingConfiguration.getActorOptimizerParameters();
        return formatParameters(actorOptimizerParameters);
    }

    public Boolean getCriticOptimizer() {
        return trainingConfiguration.hasCriticOptimizer() ? true : null;
    }

    public String getCriticOptimizerName() {
        Optional<String> criticOptimizerNameOpt = trainingConfiguration.getCriticOptimizerName();
        return criticOptimizerNameOpt.orElse(null);
    }

    public Map<String, Object> getCriticOptimizerParameters() {
        if (!getCriticOptimizer()) {
            return null;
        }
        Map<String, Object> criticOptimizerParameters = trainingConfiguration.getCriticOptimizerParameters();
        return formatParameters(criticOptimizerParameters);
    }

    public Boolean getDiscriminatorOptimizer() {
        return trainingConfiguration.hasDiscriminatorOptimizer() ? true : null;
    }

    public String getDiscriminatorOptimizerName() {
        Optional<String> discriminatorOptimizerName = trainingConfiguration.getDiscriminatorOptimizerName();
        return discriminatorOptimizerName.orElse(null);
    }

    public Map<String, Object> getDiscriminatorOptimizerParameters() {
        if (!getDiscriminatorOptimizer()) {
            return null;
        }
        Map<String, Object> discriminatorOptimizerParameters = trainingConfiguration.getDiscriminatorOptimizerParameters();
        return formatParameters(discriminatorOptimizerParameters);
    }

    public String getRetrainingType() { // TODO enum?
        Optional<String> context = trainingConfiguration.getRetrainingType();
        return context.orElse(null);
    }

    public Boolean getRetrainingOptimizer() {
        if (!trainingConfiguration.hasRetrainingOptimizer()) return null;
        return true;
    }

    public String getRetrainingOptimizerName() {
        Optional<String> getRetrainingOptimizerNameOpt = trainingConfiguration.getRetrainingOptimizerName();
        return getRetrainingOptimizerNameOpt.orElse(null);
    }

    public Map<String, Object> getRetrainingOptimizerParameters() {
        if (!getRetrainingOptimizer()) {
            return null;
        }

        return trainingConfiguration.getRetrainingOptimizerParameters();
    }

    public Boolean getNoiseDistribution() {
        return trainingConfiguration.hasNoiseDistribution() ? true : null;
    }

    public String getNoiseDistributionName() {
        Optional<String> noiseDistributionNameOpt = trainingConfiguration.getNoiseDistributionName();
        return noiseDistributionNameOpt.orElse(null);
    }

    public Map<String, Object> getNoiseDistributionParameters() {
        if (!getNoiseDistribution()) {
            return null;
        }
        Map<String, Object> noiseDistributionParameters = trainingConfiguration.getNoiseDistributionParameters();
        return formatParameters(noiseDistributionParameters);
    }

    public Boolean getEvalMetric() {
        return trainingConfiguration.hasEvalMetric() ? true : null;
    }

    public String getEvalMetricName() {
        Optional<String> evalMetricNameOpt = trainingConfiguration.getEvalMetricName();
        return evalMetricNameOpt.orElse(null);
    }

    public Map<String, Object> getEvalMetricParameters() {
        if (!getEvalMetric()) {
            return null;
        }
        Map<String, Object> evalMetricParameters = trainingConfiguration.getEvalMetricParameters();
        return formatParameters(evalMetricParameters);
    }

    public Boolean getEnvironment() {
        return trainingConfiguration.hasEnvironment() ? true : null;
    }

    public String getEnvironmentName() {
        Optional<String> environmentNameOpt = trainingConfiguration.getEnvironmentName();
        return environmentNameOpt.orElse(null);
    }

    public Map<String, Object> getEnvironmentParameters() {
        if (getEnvironment() == null || !getEnvironment()) {
            return null;
        }
        Map<String, Object> environmentParameters = trainingConfiguration.getEnvironmentParameters();
        return formatParameters(environmentParameters);
    }

    public Boolean getStrategy() {
        return trainingConfiguration.hasStrategy() ? true : null;
    }

    public String getStrategyName() {
        Optional<String> strategyNameOpt = trainingConfiguration.getStrategyName();
        return strategyNameOpt.orElse(null);
    }

    public Map<String, Object> getStrategyParameters() {
        if (!getStrategy()) {
            return null;
        }

        Map<String, Object> strategyParameters = formatParameters(trainingConfiguration.getStrategyParameters());

        ArchitectureAdapter trainedArchitecture = null;
        Optional<ArchitectureAdapter> trainedArchitectureOpt = trainingComponentsContainer.getTrainedArchitecture();
        Optional<ArchitectureAdapter> criticNetworkOpt = trainingComponentsContainer.getCriticNetwork();
        if (trainedArchitectureOpt.isPresent()) {
            trainedArchitecture = trainedArchitectureOpt.get();

        } else if (criticNetworkOpt.isPresent()) {
            trainedArchitecture = criticNetworkOpt.get();
        }

        if (trainedArchitecture != null) {
            final String actionPortName = getOutputNameOfTrainedArchitecture();
            Range actionRange = trainedArchitecture.getRanges().get(actionPortName);

            if (actionRange.isLowerLimitInfinity() && actionRange.isUpperLimitInfinity()) {
                strategyParameters.put("action_low", null);
                strategyParameters.put("action_high", null);
            } else if(!actionRange.isLowerLimitInfinity() && actionRange.isUpperLimitInfinity()) {
                assert actionRange.getLowerLimit().isPresent();
                strategyParameters.put("action_low", actionRange.getLowerLimit().get());
                strategyParameters.put("action_high", null);
            } else if (actionRange.isLowerLimitInfinity() && !actionRange.isUpperLimitInfinity()) {
                assert actionRange.getUpperLimit().isPresent();
                strategyParameters.put("action_low", null);
                strategyParameters.put("action_high", actionRange.getUpperLimit().get());
            } else {
                assert actionRange.getLowerLimit().isPresent();
                assert actionRange.getUpperLimit().isPresent();
                strategyParameters.put("action_low", actionRange.getLowerLimit().get());
                strategyParameters.put("action_high", actionRange.getUpperLimit().get());
            }
        }
        return strategyParameters;
    }

    public Boolean getReplayMemory() {
        return trainingConfiguration.hasReplayMemory() ? true : null;
    }

    public String getReplayMemoryName() {
        Optional<String> replayMemoryNameOpt = trainingConfiguration.getReplayMemoryName();
        return replayMemoryNameOpt.orElse(null);
    }

    public Map<String, Object> getReplayMemoryParameters() {
        if (!getReplayMemory()) {
            return null;
        }
        Map<String, Object> replayMemoryParameters = trainingConfiguration.getReplayMemoryParameters();
        return formatParameters(replayMemoryParameters);
    }

    public Boolean getLoss() {
        return trainingConfiguration.hasLoss() ? true : null;
    }

    public String getLossName() {
        if (!getLoss()) {
            return null;
        }
        Optional<String> lossNameOpt = trainingConfiguration.getLossName();
        return lossNameOpt.orElse(null);
    }

    public Map<String, Object> getLossParameters() {
        if (!getLoss()) {
            return null;
        }
        Map<String, Object> lossParameters = trainingConfiguration.getLossParameters();
        return formatParameters(lossParameters);
    }

//    public Map<String, Map<String, Object>> getConstraintDistributions() { // TODO
//        return getMultiParamMapEntry(CONSTRAINT_DISTRIBUTION, "name");
//    }
//
//    public Map<String, Map<String, Object>> getConstraintLosses() { // TODO
//        return getMultiParamMapEntry(CONSTRAINT_LOSS, "name");
//    }
    public String getSelfPlay() { // added Parameter self_play for cooperative driving
        Optional<String> selfPlay = trainingConfiguration.getSelfPlay();
        return selfPlay.orElse(null);
    }
    
    public String getRlAlgorithm() {
        Optional<RlAlgorithm> rlAlgorithmOpt = trainingConfiguration.getRlAlgorithm();
        if (!rlAlgorithmOpt.isPresent()) {
            return DQN;
        }

        RlAlgorithm rlAlgorithm = rlAlgorithmOpt.get();
        if (rlAlgorithm.equals(RlAlgorithm.DQN)) {
            return DQN;
        } else if (rlAlgorithm.equals(RlAlgorithm.DDPG)) {
            return DDPG;
        } else if (rlAlgorithm.equals(RlAlgorithm.TD3)) {
            return TD3;
        }
        return DQN;
    }

    public String getNetworkType() {
        Optional<NetworkType> networkTypeOpt = trainingConfiguration.getNetworkType();

        NetworkType networkType = networkTypeOpt.get();
        if (networkType.equals(NetworkType.GNN)) {
            return GNN;
        }
        return null;
    }

//    protected Object getDefaultValueOrElse(String parameterKey, Object elseValue) {
//        if (schema == null) {
//            return elseValue;
//        }
//
//        Optional<Object> defaultValueOpt = null;
//        Optional<BasicSchemaPropertySymbol> basicPropertySymbolOpt = schema.getAttributeEntryOfKindBasic(parameterKey);
//        if (basicPropertySymbolOpt.isPresent()) {
//            BasicSchemaPropertySymbol schemaPropertySymbol = basicPropertySymbolOpt.get();
//            defaultValueOpt = schemaPropertySymbol.getDefaultValue();
//
//        } else {
//            Optional<EnumSchemaPropertySymbol> enumPropertySymbolOpt = schema.getAttributeEntryOfKindEnum(parameterKey);
//            if (enumPropertySymbolOpt.isPresent()) {
//                EnumSchemaPropertySymbol enumPropertySymbol = enumPropertySymbolOpt.get();
//                defaultValueOpt = enumPropertySymbol.getDefaultValue();
//            }
//        }
//
//        if (defaultValueOpt != null && defaultValueOpt.isPresent()) {
//            return defaultValueOpt.get();
//        }
//        return elseValue;
//    }

    public Boolean getUseFixTargetNetwork() {
        Optional<Boolean> useFixTargetNetworkOpt = trainingConfiguration.getUseFixTargetNetwork();
        return useFixTargetNetworkOpt.orElse(null);
    }

    public String getSoftTargetUpdateRate() {
        Optional<Double> softTargetUpdateRateOpt = trainingConfiguration.getSoftTargetUpdateRate();
        return softTargetUpdateRateOpt.map(Object::toString).orElse(null);
    }

    public Double getPolicyNoise() {
        Optional<Double> policyNoiseOpt = trainingConfiguration.getPolicyNoise();
        return policyNoiseOpt.orElse(null);
    }

    public Integer getPolicyDelay() {
        Optional<Integer> policyDelayOpt = trainingConfiguration.getPolicyDelay();
        return policyDelayOpt.orElse(null);
    }

    public Double getNoiseClip() {
        Optional<Double> noiseClipOpt = trainingConfiguration.getNoiseClip();
        return noiseClipOpt.orElse(null);
    }

    public Integer getTargetNetworkUpdateInterval() {
        Optional<Integer> targetNetworkUpdateIntervalOpt = trainingConfiguration.getTargetNetworkUpdateInterval();
        return targetNetworkUpdateIntervalOpt.orElse(null);
    }

    public Double getDiscountFactor() {
        Optional<Double> discountFactorOpt = trainingConfiguration.getDiscountFactor();
        return discountFactorOpt.orElse(null);
    }

    public Integer getNumEpisodes() {
        Optional<Integer> numEpisodesOpt = trainingConfiguration.getNumEpisodes();
        return numEpisodesOpt.orElse(null);
    }

    public Integer getTrainingInterval() {
        Optional<Integer> trainingIntervalOpt = trainingConfiguration.getTrainingInterval();
        return trainingIntervalOpt.orElse(null);
    }

    public Integer getStartTrainingAt() {
        Optional<Integer> startTrainingAtOpt = trainingConfiguration.getStartTrainingAt();
        return startTrainingAtOpt.orElse(null);
    }

    public Integer getSnapshotInterval() {
        Optional<Integer> snapshotIntervalOpt = trainingConfiguration.getSnapshotInterval();
        return snapshotIntervalOpt.orElse(null);
    }

    public Integer getNumMaxSteps() {
        Optional<Integer> numMaxStepsOpt = trainingConfiguration.getNumMaxSteps();
        return numMaxStepsOpt.orElse(null);
    }

    public Integer getEvaluationSamples() {
        Optional<Integer> evaluationSamplesOpt = trainingConfiguration.getEvaluationSamples();
        return evaluationSamplesOpt.orElse(null);
    }

    public Integer getKValue() {
        Optional<Integer> kValueOpt = trainingConfiguration.getKValue();
        return kValueOpt.orElse(null);
    }

    public Double getGeneratorLossWeight() {
        Optional<Double> generatorLossWeightOpt = trainingConfiguration.getGeneratorLossWeight();
        return generatorLossWeightOpt.orElse(null);
    }

    public Double getDiscriminatorLossWeight() {
        Optional<Double> discriminatorLossWeightOpt = trainingConfiguration.getDiscriminatorLossWeight();
        return discriminatorLossWeightOpt.orElse(null);
    }

    public String getAgentName() {
        Optional<String> agentNameOpt = trainingConfiguration.getAgentName();
        return agentNameOpt.orElse(null);
    }

    public String getNoiseInput() {
        Optional<String> noiseInputOpt = trainingConfiguration.getNoiseInput();
        return noiseInputOpt.orElse(null);
    }

    public Boolean getPrintImages() {
        Optional<Boolean> printImagesOpt = trainingConfiguration.getPrintImages();
        return printImagesOpt.orElse(null);
    }

    public Boolean getUseDoubleDqn() {
        Optional<Boolean> useDoubleDqnOpt = trainingConfiguration.getUseDoubleDqn();
        return useDoubleDqnOpt.orElse(null);
    }

    // TODO
    public Integer getOutputDirectory() {
        return null;
    }

    public Double getTargetScore() {
        Optional<Double> targetScoreOpt = trainingConfiguration.getTargetScore();
        if (!targetScoreOpt.isPresent()) {
            return null;
        }
        return new Double(targetScoreOpt.get());
    }

    public String getInputNameOfTrainedArchitecture() {
        Optional<ArchitectureAdapter> trainedArchitectureOpt = trainingComponentsContainer.getTrainedArchitecture();
        if (!trainedArchitectureOpt.isPresent()) {
            throw new IllegalStateException("No trained architecture set");
        }

        ArchitectureAdapter trainedArchitecture = trainedArchitectureOpt.get();
        // We allow only one input, the first one is the only input
        return trainedArchitecture.getInputs().get(0);
    }

    public String getOutputNameOfTrainedArchitecture() {
        Optional<ArchitectureAdapter> trainedArchitectureOpt = trainingComponentsContainer.getTrainedArchitecture();
        if (!trainedArchitectureOpt.isPresent()) {
            throw new IllegalStateException("No trained architecture set");
        }

        ArchitectureAdapter trainedArchitecture = trainedArchitectureOpt.get();
        // We allow only one output, the first one is the only output
        return trainedArchitecture.getOutputs().get(0);
    }

    public List<Integer> getStateDim() {
        Optional<ArchitectureAdapter> trainedArchitectureOpt = trainingComponentsContainer.getTrainedArchitecture();
        if (!trainedArchitectureOpt.isPresent()) {
            return null;
        }

        final String inputName = getInputNameOfTrainedArchitecture();
        ArchitectureAdapter trainedArchitecture = trainedArchitectureOpt.get();
        return trainedArchitecture.getDimensions().get(inputName);
    }

    public Boolean isDiscreteState() {
        Optional<ArchitectureAdapter> trainedArchitectureOpt = trainingComponentsContainer.getTrainedArchitecture();
        if (!trainedArchitectureOpt.isPresent()) {
            return null;
        }

        final String inputName = getInputNameOfTrainedArchitecture();
        ArchitectureAdapter trainedArchitecture = trainedArchitectureOpt.get();
        
        String stateType = trainedArchitecture.getTypes().get(inputName);
        return stateType.equals("Z");
    }

    public List<Integer> getActionDim() {
        Optional<ArchitectureAdapter> trainedArchitectureOpt = trainingComponentsContainer.getTrainedArchitecture();
        if (!trainedArchitectureOpt.isPresent()) {
            return null;
        }

        final String outputName = getOutputNameOfTrainedArchitecture();
        ArchitectureAdapter trainedArchitecture = trainedArchitectureOpt.get();
        return trainedArchitecture.getDimensions().get(outputName);
    }

    public Boolean hasRewardFunction() {
        return trainingConfiguration.hasRewardFunction();
    }

    public String getRewardFunctionName() {
        Optional<String> rewardFunctionNameOpt = trainingConfiguration.getRewardFunctionName();
        return rewardFunctionNameOpt.orElse(null);
    }

    public String getRewardFunctionComponentName() {
        Optional<String> rewardFunctionNameOpt = trainingConfiguration.getRewardFunctionName();
        if (rewardFunctionNameOpt.isPresent()) {
            String rewardFunctionName = rewardFunctionNameOpt.get();
            String rewardFunctionComponentName = rewardFunctionName.replaceAll("\\.", "_");
            return rewardFunctionComponentName;
        }
        return null;
    }

    public boolean isDiscreteRlAlgorithm() {
        return isReinforcementLearning() && getRlAlgorithm().equals(DQN);
    }

    public boolean isContinuousRlAlgorithm() {
        return isReinforcementLearning() && getRlAlgorithm().equals(DDPG);
    }

    public boolean hasRosRewardTopic() {
        Map<String, Object> environmentParameters = getEnvironmentParameters();
        if (environmentParameters == null) {
            return false;
        }
        return environmentParameters.containsKey(ENVIRONMENT_REWARD_TOPIC);
    }

    public String getInstanceName() {
        return instanceName;
    }

    // TODO Refactor template files to do the formatting
    private Map<String, Object> formatParameters(Map<String, Object> parameterMap) {
        if (parameterMap == null || parameterMap.isEmpty()) {
            return parameterMap;
        }

        Map<String, Object> formattedParameterMap = Maps.newHashMap();
        for (Map.Entry<String, Object> entry : parameterMap.entrySet()) {
            formattedParameterMap.put(entry.getKey(), formatValue(entry.getValue()));
        }
        return formattedParameterMap;
    }

    private static Object formatValue(Object object) {
        if (object instanceof Integer) {
            Integer value = (Integer) object;
            return value.toString();

        } else if (object instanceof Double) {
            Double value = (Double) object;
            return value.toString();

        } else if (object instanceof Boolean) {
            Boolean value = (Boolean) object;
            return value ? "True" : "False";

        } else if (object instanceof String) {
            String value = (String) object;
            return value;

        } else if (object instanceof List) {
            List<Object> list = (List<Object>) object;
            List<Object> vectorEntries = Lists.newArrayList();
            for (Object literal : list) {
                vectorEntries.add(formatValue(literal));
            }
            return vectorEntries;
        }
        return object.toString();
    }
}
