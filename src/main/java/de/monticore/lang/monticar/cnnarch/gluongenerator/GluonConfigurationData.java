package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionParameterAdapter;
import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;
import de.monticore.lang.monticar.cnntrain._symboltable.*;
import de.monticore.lang.monticar.cnntrain.annotations.Range;

import static de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants.*;

import java.util.*;

public class GluonConfigurationData extends ConfigurationData {
    public GluonConfigurationData(ConfigurationSymbol configuration, String instanceName) {
        super(configuration, instanceName);
    }

    public Boolean isSupervisedLearning() {
        if (configurationContainsKey(LEARNING_METHOD)) {
            return retrieveConfigurationEntryValueByKey(LEARNING_METHOD)
                    .equals(LearningMethod.SUPERVISED);
        }
        return true;
    }

    public Boolean isReinforcementLearning() {
        return configurationContainsKey(LEARNING_METHOD)
                && retrieveConfigurationEntryValueByKey(LEARNING_METHOD).equals(LearningMethod.REINFORCEMENT);
    }

    public Integer getNumEpisodes() {
        return !configurationContainsKey(NUM_EPISODES)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(NUM_EPISODES);
    }

    public Double getDiscountFactor() {
        return !configurationContainsKey(DISCOUNT_FACTOR)
                ? null : (Double)retrieveConfigurationEntryValueByKey(DISCOUNT_FACTOR);
    }

    public Integer getNumMaxSteps() {
        return !configurationContainsKey(NUM_MAX_STEPS)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(NUM_MAX_STEPS);
    }

    public Double getTargetScore() {
        return !configurationContainsKey(TARGET_SCORE)
                ? null : (Double)retrieveConfigurationEntryValueByKey(TARGET_SCORE);
    }

    public Integer getTrainingInterval() {
        return !configurationContainsKey(TRAINING_INTERVAL)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(TRAINING_INTERVAL);
    }

    public Boolean getUseFixTargetNetwork() {
        return !configurationContainsKey(USE_FIX_TARGET_NETWORK)
                ? null : (Boolean)retrieveConfigurationEntryValueByKey(USE_FIX_TARGET_NETWORK);
    }

    public Integer getTargetNetworkUpdateInterval() {
        return !configurationContainsKey(TARGET_NETWORK_UPDATE_INTERVAL)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(TARGET_NETWORK_UPDATE_INTERVAL);
    }

    public Integer getSnapshotInterval() {
        return !configurationContainsKey(SNAPSHOT_INTERVAL)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(SNAPSHOT_INTERVAL);
    }

    public String getAgentName() {
        return !configurationContainsKey(AGENT_NAME)
                ? null : (String)retrieveConfigurationEntryValueByKey(AGENT_NAME);
    }

    public Boolean getUseDoubleDqn() {
        return !configurationContainsKey(USE_DOUBLE_DQN)
                ? null : (Boolean)retrieveConfigurationEntryValueByKey(USE_DOUBLE_DQN);
    }

    public Double getSoftTargetUpdateRate() {
        return !configurationContainsKey(SOFT_TARGET_UPDATE_RATE)
                ? null : (Double)retrieveConfigurationEntryValueByKey(SOFT_TARGET_UPDATE_RATE);
    }

    public Integer getStartTrainingAt() {
        return !configurationContainsKey(START_TRAINING_AT)
            ? null : (Integer)retrieveConfigurationEntryValueByKey(START_TRAINING_AT);
    }

    public Integer getEvaluationSamples() {
        return !configurationContainsKey(EVALUATION_SAMPLES)
            ? null : (Integer)retrieveConfigurationEntryValueByKey(EVALUATION_SAMPLES);
    }

    public Double getPolicyNoise() {
        return !configurationContainsKey(POLICY_NOISE)
            ? null : (Double) retrieveConfigurationEntryValueByKey(POLICY_NOISE);
    }

    public Double getNoiseClip() {
        return !configurationContainsKey(NOISE_CLIP)
            ? null : (Double) retrieveConfigurationEntryValueByKey(NOISE_CLIP);
    }

    public Integer getPolicyDelay() {
        return !configurationContainsKey(POLICY_DELAY)
            ? null : (Integer) retrieveConfigurationEntryValueByKey(POLICY_DELAY);
    }

    public RLAlgorithm getRlAlgorithm() {
        if (!isReinforcementLearning()) {
            return null;
        }
        return !configurationContainsKey(RL_ALGORITHM)
            ? RLAlgorithm.DQN : (RLAlgorithm)retrieveConfigurationEntryValueByKey(RL_ALGORITHM);
    }

    public String getInputNameOfTrainedArchitecture() {
        if (!this.getConfiguration().getTrainedArchitecture().isPresent()) {
            throw new IllegalStateException("No trained architecture set");
        }
        NNArchitectureSymbol trainedArchitecture = getConfiguration().getTrainedArchitecture().get();
        // We allow only one input, the first one is the only input
        return trainedArchitecture.getInputs().get(0);
    }

    public String getOutputNameOfTrainedArchitecture() {
        if (!this.getConfiguration().getTrainedArchitecture().isPresent()) {
            throw new IllegalStateException("No trained architecture set");
        }
        NNArchitectureSymbol trainedArchitecture = getConfiguration().getTrainedArchitecture().get();
        // We allow only one output, the first one is the only output
        return trainedArchitecture.getOutputs().get(0);
    }

    public List<Integer> getStateDim() {
        if (!this.getConfiguration().getTrainedArchitecture().isPresent()) {
            return null;
        }
        final String inputName = getInputNameOfTrainedArchitecture();
        NNArchitectureSymbol trainedArchitecture = this.getConfiguration().getTrainedArchitecture().get();
        return trainedArchitecture.getDimensions().get(inputName);
    }

    public List<Integer> getActionDim() {
        if (!this.getConfiguration().getTrainedArchitecture().isPresent()) {
            return null;
        }
        final String outputName = getOutputNameOfTrainedArchitecture();
        NNArchitectureSymbol trainedArchitecture = this.getConfiguration().getTrainedArchitecture().get();
        return trainedArchitecture.getDimensions().get(outputName);
    }

    public String getLoss() {
        return !configurationContainsKey(LOSS)
                ? null : retrieveConfigurationEntryValueByKey(LOSS).toString();
    }

    public Map<String, Object> getReplayMemory() {
        return getMultiParamEntry(REPLAY_MEMORY, "method");
    }

    public Map<String, Object> getStrategy() {
        assert isReinforcementLearning(): "Strategy parameter only for reinforcement learning but called in a " +
         " non reinforcement learning context";
        Map<String, Object> strategyParams = getMultiParamEntry(STRATEGY, "method");
        assert getConfiguration().getTrainedArchitecture().isPresent(): "Architecture not present," +
         " but reinforcement training";
        NNArchitectureSymbol trainedArchitecture = getConfiguration().getTrainedArchitecture().get();
        final String actionPortName = getOutputNameOfTrainedArchitecture();
        Range actionRange = trainedArchitecture.getRanges().get(actionPortName);

        if (actionRange.isLowerLimitInfinity() && actionRange.isUpperLimitInfinity()) {
            strategyParams.put("action_low", null);
            strategyParams.put("action_high", null);
        } else if(!actionRange.isLowerLimitInfinity() && actionRange.isUpperLimitInfinity()) {
            assert actionRange.getLowerLimit().isPresent();
            strategyParams.put("action_low", actionRange.getLowerLimit().get());
            strategyParams.put("action_high", null);
        } else if (actionRange.isLowerLimitInfinity() && !actionRange.isUpperLimitInfinity()) {
            assert actionRange.getUpperLimit().isPresent();
            strategyParams.put("action_low", null);
            strategyParams.put("action_high", actionRange.getUpperLimit().get());
        } else {
            assert actionRange.getLowerLimit().isPresent();
            assert actionRange.getUpperLimit().isPresent();
            strategyParams.put("action_low", actionRange.getLowerLimit().get());
            strategyParams.put("action_high", actionRange.getUpperLimit().get());
        }
        return strategyParams;
    }

    public Map<String, Object> getEnvironment() {
        return getMultiParamEntry(ENVIRONMENT, "environment");
    }

    public Boolean hasRewardFunction() {
        return this.getConfiguration().getRlRewardFunction().isPresent();
    }

    public String getRewardFunctionName() {
        if (!this.getConfiguration().getRlRewardFunction().isPresent()) {
            return null;
        }
        return String.join("_", this.getConfiguration().getRlRewardFunction()
                .get().getRewardFunctionComponentName());
    }

    private Optional<RewardFunctionParameterAdapter> getRlRewardFunctionParameter() {
        if (!this.getConfiguration().getRlRewardFunction().isPresent()
                || !this.getConfiguration().getRlRewardFunction().get().getRewardFunctionParameter().isPresent()) {
            return Optional.empty();
        }
        return Optional.ofNullable(
                (RewardFunctionParameterAdapter)this.getConfiguration().getRlRewardFunction().get()
                        .getRewardFunctionParameter().orElse(null));
    }

    public boolean isDiscreteRlAlgorithm() {
        assert isReinforcementLearning();
        return getRlAlgorithm().equals(RLAlgorithm.DQN);
    }

    public boolean isContinuousRlAlgorithm() {
        assert isReinforcementLearning();
        return getRlAlgorithm().equals(RLAlgorithm.DDPG);
    }

    public Map<String, Object> getRewardFunctionStateParameter() {
        if (!getRlRewardFunctionParameter().isPresent()
            || !getRlRewardFunctionParameter().get().getInputStateParameterName().isPresent()) {
            return null;
        }
        return getInputParameterWithName(getRlRewardFunctionParameter().get().getInputStateParameterName().get());
    }

    public Map<String, Object> getRewardFunctionTerminalParameter() {
        if (!getRlRewardFunctionParameter().isPresent()
                || !getRlRewardFunctionParameter().get().getInputTerminalParameter().isPresent()) {
            return null;
        }
        return getInputParameterWithName(getRlRewardFunctionParameter().get().getInputTerminalParameter().get());
    }

    public String getRewardFunctionOutputName() {
        if (!getRlRewardFunctionParameter().isPresent()) {
            return null;
        }
        return getRlRewardFunctionParameter().get().getOutputParameterName().orElse(null);
    }

    public String getCriticOptimizerName() {
        if (!getConfiguration().getCriticOptimizer().isPresent()) {
            return null;
        }
        return getConfiguration().getCriticOptimizer().get().getName();
    }

    public Map<String, String> getCriticOptimizerParams() {
        // get classes for single enum values
        assert getConfiguration().getCriticOptimizer().isPresent():
            "Critic optimizer params called although, not present";
        List<Class> lrPolicyClasses = new ArrayList<>();
        for (LRPolicy enum_value: LRPolicy.values()) {
            lrPolicyClasses.add(enum_value.getClass());
        }



        Map<String, String>  mapToStrings = new HashMap<>();
        Map<String, OptimizerParamSymbol> optimizerParams =
            getConfiguration().getCriticOptimizer().get().getOptimizerParamMap();
        for (Map.Entry<String, OptimizerParamSymbol> entry : optimizerParams.entrySet()) {
            String paramName = entry.getKey();
            String valueAsString = entry.getValue().toString();
            Class realClass = entry.getValue().getValue().getValue().getClass();
            if (realClass == Boolean.class) {
                valueAsString = (Boolean) entry.getValue().getValue().getValue() ? "True" : "False";
            } else if (lrPolicyClasses.contains(realClass)) {
                valueAsString = "'" + valueAsString + "'";
            }
            mapToStrings.put(paramName, valueAsString);
        }
        return mapToStrings;
    }

    public boolean hasRosRewardTopic() {
        Map<String, Object> environmentParameters = getMultiParamEntry(ENVIRONMENT, "environment");
        if (environmentParameters == null
            || !environmentParameters.containsKey("environment")) {
            return false;
        }
        return environmentParameters.containsKey(ENVIRONMENT_REWARD_TOPIC);
    }

    private Map<String, Object> getMultiParamEntry(final String key, final String valueName) {
        if (!configurationContainsKey(key)) {
            return null;
        }

        Map<String, Object> resultView = new HashMap<>();

        MultiParamValueSymbol multiParamValue = (MultiParamValueSymbol)this.getConfiguration().getEntryMap()
                .get(key).getValue();

        resultView.put(valueName, multiParamValue.getValue());
        resultView.putAll(multiParamValue.getParameters());
        return resultView;
    }

    private Boolean configurationContainsKey(final String key) {
        return this.getConfiguration().getEntryMap().containsKey(key);
    }

    private Object retrieveConfigurationEntryValueByKey(final String key) {
        return this.getConfiguration().getEntry(key).getValue().getValue();
    }

    private Map<String, Object> getInputParameterWithName(final String parameterName) {
        if (!getRlRewardFunctionParameter().isPresent()
                || !getRlRewardFunctionParameter().get().getTypeOfInputPort(parameterName).isPresent()
                || !getRlRewardFunctionParameter().get().getInputPortDimensionOfPort(parameterName).isPresent()) {
            return null;
        }

        Map<String, Object> functionStateParameter = new HashMap<>();;

        final String portType = getRlRewardFunctionParameter().get().getTypeOfInputPort(parameterName).get();
        final List<Integer> dimension = getRlRewardFunctionParameter().get().getInputPortDimensionOfPort(parameterName).get();

        String dtype = null;
        if (portType.equals("Q")) {
            dtype = "double";
        } else if (portType.equals("Z")) {
            dtype = "int";
        } else if (portType.equals("B")) {
            dtype = "bool";
        }

        Boolean isMultiDimensional = dimension.size() > 1
                || (dimension.size() == 1 && dimension.get(0) > 1);


        functionStateParameter.put("name", parameterName);
        functionStateParameter.put("dtype", dtype);
        functionStateParameter.put("isMultiDimensional", isMultiDimensional);

        return functionStateParameter;
    }
}
