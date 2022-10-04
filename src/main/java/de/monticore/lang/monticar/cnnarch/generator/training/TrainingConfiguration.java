package de.monticore.lang.monticar.cnnarch.generator.training;

import com.google.common.collect.Maps;
import conflang._symboltable.ConfigurationSymbol;
import conflang._symboltable.ConfigurationEntry;
import conflang._symboltable.ConfigurationEntrySymbol;
import conflang._symboltable.NestedConfigurationEntrySymbol;
import schemalang._symboltable.SchemaDefinitionSymbol;

import java.util.*;

import static de.monticore.lang.monticar.cnnarch.generator.training.TrainingParameterConstants.*;

/**
 * Encapsulates the access to the training configuration.
 */
public class TrainingConfiguration {

    private final ConfigurationSymbol configurationSymbol;
    private Collection<SchemaDefinitionSymbol> schemaDefinitions;

    private TrainingConfiguration(ConfigurationSymbol configurationSymbol) {
        if (configurationSymbol == null) throw new IllegalArgumentException("The configuration must not be null");
        this.configurationSymbol = configurationSymbol;
    }

    public static TrainingConfiguration create(ConfigurationSymbol configurationSymbol) {
        return new TrainingConfiguration(configurationSymbol);
    }

    public static TrainingConfiguration create(ConfigurationSymbol configurationSymbol,
                                               Collection<SchemaDefinitionSymbol> schemaDefinitions) {
        TrainingConfiguration configuration = create(configurationSymbol);
        configuration.setSchemaDefinitions(schemaDefinitions);
        return configuration;
    }

    public void setSchemaDefinitions(Collection<SchemaDefinitionSymbol> schemaDefinitions) {
        this.schemaDefinitions = schemaDefinitions;
    }

    public ConfigurationSymbol getConfigurationSymbol() {
        return configurationSymbol;
    }

    public Optional<String> getContext() {
        return getParameterValue(CONTEXT);
    }

    public Optional<String> getSelfPlay() {
        return getParameterValue(SELF_PLAY);
    }

    public Optional<LearningMethod> getLearningMethod() {
        Optional<ConfigurationEntry> learningMethodOpt =
                configurationSymbol.getConfigurationEntry(LEARNING_METHOD);
        if (!learningMethodOpt.isPresent()) {
            Optional<String> defaultValueOpt = getDefaultValue(LEARNING_METHOD);
            if (defaultValueOpt.isPresent()) {
                return Optional.of(LearningMethod.learningMethod(defaultValueOpt.get()));
            }
            return Optional.empty();
        }
        ConfigurationEntry configurationEntry = learningMethodOpt.get();
        String learningMethod = (String) configurationEntry.getValue();
        return Optional.of(LearningMethod.learningMethod(learningMethod));
    }

    public boolean isSupervisedLearning() {
        Optional<LearningMethod> learningMethodOpt = getLearningMethod();
        if (!learningMethodOpt.isPresent()) {
            return true; // default is supervised
        }
        LearningMethod learningMethod = learningMethodOpt.get();
        return LearningMethod.SUPERVISED.equals(learningMethod);
    }

    public boolean isGanLearning() {
        Optional<LearningMethod> learningMethodOpt = getLearningMethod();
        if (!learningMethodOpt.isPresent()) {
            return false; // Not correct here to return false..
        }
        LearningMethod learningMethod = learningMethodOpt.get();
        return LearningMethod.GAN.equals(learningMethod);
    }

    public boolean isVaeLearning() {
        Optional<LearningMethod> learningMethodOpt = getLearningMethod();
        if (!learningMethodOpt.isPresent()) {
            return false; // Not correct here to return false..
        }
        LearningMethod learningMethod = learningMethodOpt.get();
        return LearningMethod.VAE.equals(learningMethod);
    }

    public boolean isReinforcementLearning() {
        Optional<LearningMethod> learningMethodOpt = getLearningMethod();
        if (!learningMethodOpt.isPresent()) {
            return false; // Not correct here to return false..
        }
        LearningMethod learningMethod = learningMethodOpt.get();
        return LearningMethod.REINFORCEMENT.equals(learningMethod);
    }

    public Optional<RlAlgorithm> getRlAlgorithm() {
        Optional<ConfigurationEntry> rlAlgorithmOpt = configurationSymbol.getConfigurationEntry(RL_ALGORITHM);
        if (!rlAlgorithmOpt.isPresent()) {
            return Optional.empty();
        }
        String rlAlgorithm = (String) rlAlgorithmOpt.get().getValue();
        return Optional.of(RlAlgorithm.rlAlgorithm(rlAlgorithm));
    }

    public Optional<NetworkType> getNetworkType() {
        Optional<ConfigurationEntry> networkTypeOpt = configurationSymbol.getConfigurationEntry(NETWORK_TYPE);
        if (!networkTypeOpt.isPresent()) {
            return Optional.empty();
        }
        String networkType = (String) networkTypeOpt.get().getValue();
        return Optional.of(NetworkType.networkType(networkType));
    }

    public Optional<Integer> getBatchSize() {
        return getParameterValue(BATCH_SIZE);
    }

    public Optional<Integer> getNumEpoch() {
        return getParameterValue(NUM_EPOCH);
    }

    public Optional<Boolean> getLoadCheckpoint() {
        return getParameterValue(LOAD_CHECKPOINT);
    }

    public Optional<Integer> getCheckpointPeriod() {
        return getParameterValue(CHECKPOINT_PERIOD);
    }

    public Optional<Integer> getLogPeriod() {
        return getParameterValue(LOG_PERIOD);
    }

    public Optional<Boolean> getLoadPretrained() {
        return getParameterValue(LOAD_PRETRAINED);
    }

    public Optional<String> getPreprocessor() {
        return getParameterValue(PREPROCESSING_NAME);
    }

    public Optional<Boolean> getNormalize() {
        return getParameterValue(NORMALIZE);
    }

    public Optional<Boolean> getOnnxExport() {
        return getParameterValue(ONNX_EXPORT);
    }

    public boolean hasCleaning() {
        return hasParameter(CLEANING);
    }

    public Optional<String> getCleaningName() {
        return getObjectParameterValue(CLEANING);
    }

    public Map<String, Object> getCleaningParameters() {
        return getObjectParameterParameters(CLEANING);
    }

    public boolean hasDataImbalance() {
        return hasParameter(DATA_IMBALANCE);
    }

    public Optional<String> getDataImbalanceName() {
        return getObjectParameterValue(DATA_IMBALANCE);
    }

    public Map<String, Object> getDataImbalanceParameters() {
        return getObjectParameterParameters(DATA_IMBALANCE);
    }

    public boolean hasDataSplitting() {
        return hasParameter(DATA_SPLITTING);
    }

    public Optional<String> getDataSplittingName() {
        return getObjectParameterValue(DATA_SPLITTING);
    }

    public Map<String, Object> getDataSplittingParameters() {
        return getObjectParameterParameters(DATA_SPLITTING);
    }
    
    public Optional<Boolean> getShuffleData() {
        return getParameterValue(SHUFFLE_DATA);
    }

    public Optional<Boolean> getMultiGraph() {
        return getParameterValue(MULTI_GRAPH);
    }

    public Optional<List<Integer>> getTrainMask() {
        return getParameterValue(TRAIN_MASK);
    }

    public Optional<List<Integer>> getTestMask() {
        return getParameterValue(TEST_MASK);
    }

    public Optional<Double> getClipGlobalGradNorm() {
        return getParameterValue(CLIP_GLOBAL_GRAD_NORM);
    }

    public Optional<Boolean> getUseTeacherForcing() {
        return getParameterValue(USE_TEACHER_FORCING);
    }

    public Optional<Boolean> getSaveAttentionImage() {
        return getParameterValue(SAVE_ATTENTION_IMAGE);
    }

    public Optional<Boolean> getEvalTrain() {
        return getParameterValue(EVAL_TRAIN);
    }

    public Optional<Boolean> getUseFixTargetNetwork() {
        return getParameterValue(USE_FIX_TARGET_NETWORK);
    }

    public Optional<Double> getSoftTargetUpdateRate() {
        return getParameterValue(SOFT_TARGET_UPDATE_RATE);
    }

    public Optional<Double> getPolicyNoise() {
        return getParameterValue(POLICY_NOISE);
    }

    public Optional<Integer> getPolicyDelay() {
        return getParameterValue(POLICY_DELAY);
    }

    public Optional<Double> getNoiseClip() {
        return getParameterValue(NOISE_CLIP);
    }

    public Optional<Integer> getTargetNetworkUpdateInterval() {
        return getParameterValue(TARGET_NETWORK_UPDATE_INTERVAL);
    }

    public Optional<Double> getDiscountFactor() {
        return getParameterValue(DISCOUNT_FACTOR);
    }

    public Optional<Integer> getNumEpisodes() {
        return getParameterValue(NUM_EPISODES);
    }

    public Optional<Integer> getTrainingInterval() {
        return getParameterValue(TRAINING_INTERVAL);
    }

    public Optional<Integer> getStartTrainingAt() {
        return getParameterValue(START_TRAINING_AT);
    }

    public Optional<Integer> getSnapshotInterval() {
        return getParameterValue(SNAPSHOT_INTERVAL);
    }

    public Optional<Integer> getNumMaxSteps() {
        return getParameterValue(NUM_MAX_STEPS);
    }

    public Optional<Integer> getEvaluationSamples() {
        return getParameterValue(EVALUATION_SAMPLES);
    }

    public Optional<Integer> getKValue() {
        return getParameterValue(K_VALUE);
    }

    public Optional<Double> getGeneratorLossWeight() {
        return getParameterValue(GENERATOR_LOSS_WEIGHT);
    }

    public Optional<Double> getDiscriminatorLossWeight() {
        return getParameterValue(DISCRIMINATOR_LOSS_WEIGHT);
    }

    public Optional<String> getAgentName() {
        return getParameterValue(AGENT_NAME);
    }

    public Optional<String> getNoiseInput() {
        return getParameterValue(NOISE_INPUT);
    }

    public Optional<Boolean> getPrintImages() {
        return getParameterValue(PRINT_IMAGES);
    }

    public Optional<Boolean> getUseDoubleDqn() {
        return getParameterValue(USE_DOUBLE_DQN);
    }

    public Optional<Double> getTargetScore() {
        return getParameterValue(TARGET_SCORE);
    }

    public Boolean hasLoss() {
        return hasParameter(LOSS);
    }

    public Optional<String> getLossName() {
        return getObjectParameterValue(LOSS);
    }

    public Map<String, Object> getLossParameters() {
        return getObjectParameterParameters(LOSS);
    }

    public boolean hasNoiseDistribution() {
        return hasParameter(NOISE_DISTRIBUTION);
    }

    public Optional<String> getNoiseDistributionName() {
        return getObjectParameterValue(NOISE_DISTRIBUTION);
    }

    public Map<String, Object> getNoiseDistributionParameters() {
        return getObjectParameterParameters(NOISE_DISTRIBUTION);
    }

    public Boolean hasEvalMetric() {
        return hasParameter(EVAL_METRIC);
    }

    public Optional<String> getEvalMetricName() {
        return getObjectParameterValue(EVAL_METRIC);
    }

    public Map<String, Object> getEvalMetricParameters() {
        return getObjectParameterParameters(EVAL_METRIC);
    }

    public Boolean hasReplayMemory() {
        return hasParameter(REPLAY_MEMORY);
    }

    public Optional<String> getReplayMemoryName() {
        return getObjectParameterValue(REPLAY_MEMORY);
    }

    public Map<String, Object> getReplayMemoryParameters() {
        return getObjectParameterParameters(REPLAY_MEMORY);
    }

    public boolean hasCritic() {
        return hasParameter(CRITIC);
    }

    public Optional<String> getCritic() {
        return getObjectParameterValue(CRITIC);
    }

    public boolean hasOptimizer() {
        return hasParameter(OPTIMIZER);
    }

    public Optional<String> getOptimizerName() {
        return getObjectParameterValue(OPTIMIZER);
    }

    public Map<String, Object> getOptimizerParameters() {
        return getObjectParameterParameters(OPTIMIZER);
    }

    public Boolean hasActorOptimizer() {
        return hasParameter(ACTOR_OPTIMIZER);
    }

    public Optional<String> getActorOptimizerName() {
        return getObjectParameterValue(ACTOR_OPTIMIZER);
    }

    public Map<String, Object> getActorOptimizerParameters() {
        return getObjectParameterParameters(ACTOR_OPTIMIZER);
    }

    public boolean hasCriticOptimizer() {
        return hasParameter(CRITIC_OPTIMIZER);
    }

    public Optional<String> getCriticOptimizerName() {
        return getObjectParameterValue(CRITIC_OPTIMIZER);
    }

    public Map<String, Object> getCriticOptimizerParameters() {
        return getObjectParameterParameters(CRITIC_OPTIMIZER);
    }

    public Optional<String> getRetrainingType() {
        return getParameterValue(RETRAINING_TYPE);
    }

    public Optional<String> getRetrainingOptimizerName() {
        return getObjectParameterValue(RETRAINING_OPTIMIZER);
    }

    public boolean hasRetrainingOptimizer() {
        return hasParameter(RETRAINING_OPTIMIZER);
    }

    public Map<String, Object> getRetrainingOptimizerParameters() {
        return getObjectParameterParameters(RETRAINING_OPTIMIZER);
    }

    public Boolean hasDiscriminatorName() {
        return hasParameter(DISCRIMINATOR);
    }

    public Optional<String> getDiscriminatorName() {
        return getObjectParameterValue(DISCRIMINATOR);
    }

    public Boolean hasDiscriminatorOptimizer() {
        return hasParameter(DISCRIMINATOR_OPTIMIZER);
    }

    public Optional<String> getDiscriminatorOptimizerName() {
        return getObjectParameterValue(DISCRIMINATOR_OPTIMIZER);
    }

    public Map<String, Object> getDiscriminatorOptimizerParameters() {
        return getObjectParameterParameters(DISCRIMINATOR_OPTIMIZER);
    }

    public Boolean hasEncoderName() {
        return hasParameter(ENCODER);
    }

    public Optional<String> getEncoderName() {
        return getObjectParameterValue(ENCODER);
    }

    public Optional<String> getReconLossName() {
        return getObjectParameterValue(RECON_LOSS);
    }

    public Optional<Double> getKlLossWeight() { return getParameterValue(KL_LOSS_WEIGHT); }

    public boolean hasStrategy() {
        return hasParameter(STRATEGY);
    }

    public Optional<String> getStrategyName() {
        return getObjectParameterValue(STRATEGY);
    }

    public Map<String, Object> getStrategyParameters() {
        return getObjectParameterParameters(STRATEGY);
    }

    public Optional<Hyperparameter> getStrategyParameter(String parameterKey) {
        return getHyperparameterFromNestedConfiguration(STRATEGY, parameterKey);
    }

    public boolean hasEnvironment() {
        return hasParameter(ENVIRONMENT);
    }

    public Optional<String> getEnvironmentName() {
        return getObjectParameterValue(ENVIRONMENT);
    }

    public Map<String, Object> getEnvironmentParameters() {
        return getObjectParameterParameters(ENVIRONMENT);
    }

    public Optional<Hyperparameter> getEnvironmentParameter(String parameterKey) {
        return getHyperparameterFromNestedConfiguration(ENVIRONMENT, parameterKey);
    }

    public boolean hasRewardFunction() {
        return hasParameter(REWARD_FUNCTION);
    }

    public Optional<String> getRewardFunctionName() {
        return getObjectParameterValue(REWARD_FUNCTION);
    }

    public Optional<String> getPolicyFunctionName() {
        return getObjectParameterValue(POLICY_FUNCTION);
    }

    public boolean hasConstraintLosses() {
        return hasParameter(CONSTRAINT_LOSS);
    }

    public boolean hasConstraintDistribution() {
        return hasParameter(CONSTRAINT_DISTRIBUTION);
    }

    public boolean hasQNetwork() {
        return hasParameter(QNETWORK);
    }

    public Optional<String> getQNetwork() {
        return getObjectParameterValue(QNETWORK);
    }

    public boolean hasRosEnvironment() {
        if (!hasEnvironment()) {
            return false;
        }
        Optional<String> environmentNameOpt = getEnvironmentName();
        return environmentNameOpt.filter(ENVIRONMENT_ROS::equals).isPresent();
    }

    public boolean hasRewardTopic() {
        Optional<Hyperparameter> rewardTopicOpt = getEnvironmentParameter(ENVIRONMENT_REWARD_TOPIC);
        return rewardTopicOpt.isPresent();
    }

    public boolean hasNoiseInput() {
        return hasParameter(NOISE_INPUT);
    }

    public boolean hasGeneratorLoss() {
        return hasParameter(GENERATOR_LOSS);
    }

    public boolean hasGeneratorTargetName() {
        return hasParameter(GENERATOR_TARGET_NAME);
    }

    /* ===================================== Helper methods ===================================== */

    private Optional<Hyperparameter> getHyperparameterFromNestedConfiguration(String nestedParameterKey, String parameterKey) {
        Optional<NestedConfigurationEntrySymbol> strategyOpt = configurationSymbol.getConfigurationEntryOfKind(nestedParameterKey, NestedConfigurationEntrySymbol.KIND);
        if (!strategyOpt.isPresent()) {
            return Optional.empty();
        }
        NestedConfigurationEntrySymbol strategy = strategyOpt.get();
        Optional<ConfigurationEntry> parameterOpt = strategy.getConfigurationEntry(parameterKey);
        if (!parameterOpt.isPresent()) {
            return Optional.empty();
        }
        ConfigurationEntry configurationEntry = parameterOpt.get();
        return Optional.of(new Hyperparameter(configurationEntry.getName(), configurationEntry.getValue()));
    }

    private <T> Optional<T> getParameterValue(String parameterKey) {
        Optional<ConfigurationEntry> parameterOpt = configurationSymbol.getConfigurationEntry(parameterKey);
        Optional<T> valueOpt = parameterOpt.map(configurationEntry -> (T) configurationEntry.getValue());
        if (!valueOpt.isPresent()) {
            valueOpt = getDefaultValue(parameterKey);
        }
        return valueOpt;
    }

    private Optional<String> getObjectParameterValue(String parameterKey) {
        Optional<ConfigurationEntry> parameterOpt = configurationSymbol.getConfigurationEntry(parameterKey);
        return parameterOpt.map(configurationEntry -> (String) configurationEntry.getValue());
    }

    private boolean hasParameter(String parameterKey) {
        Optional<ConfigurationEntry> parameterOpt = configurationSymbol.getConfigurationEntry(parameterKey);
        if (!parameterOpt.isPresent()) {
            return false;
        }
        return true;
    }

    private <T> Optional<T> getDefaultValue(String parameter) {
        if (schemaDefinitions == null || schemaDefinitions.isEmpty()) {
            return Optional.empty();
        }

        for (SchemaDefinitionSymbol schemaDefinition : schemaDefinitions) {
            if (schemaDefinition.hasDeclaration(parameter)) {
                Optional<T> defaultValueOpt = schemaDefinition.getDefaultValue(parameter);
                if (defaultValueOpt.isPresent()) return defaultValueOpt;
            }
        }
        return Optional.empty();
    }

    private Map<String, Object> getObjectParameterParameters(String parameterKey) {

        Optional<ConfigurationEntry> configurationEntryOpt = configurationSymbol.getConfigurationEntry(parameterKey);
        if (!configurationEntryOpt.isPresent()) {
            return Maps.newHashMap();
        }

        ConfigurationEntry configurationEntry = configurationEntryOpt.get();
        if (!configurationEntry.isOfSymbolKind(NestedConfigurationEntrySymbol.KIND)) {
            return null;
        }

        NestedConfigurationEntrySymbol nestedParameter = (NestedConfigurationEntrySymbol) configurationEntry;
        List<ConfigurationEntry> allConfigurationEntries = nestedParameter.getAllConfigurationEntries();

        Map<String, Object> keyValues = Maps.newHashMap();
        for (ConfigurationEntry entry : allConfigurationEntries) {
            if (entry.isOfSymbolKind(ConfigurationEntrySymbol.KIND)) {
                keyValues.put(entry.getName(), entry.getValue());
            }
        }
        return keyValues;
    }

}