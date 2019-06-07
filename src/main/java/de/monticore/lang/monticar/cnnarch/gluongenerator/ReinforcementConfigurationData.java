package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionParameterAdapter;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.ConfigurationData;
import de.monticore.lang.monticar.cnntrain._symboltable.*;
import de.monticore.lang.monticar.cnntrain.annotations.Range;
import de.monticore.lang.monticar.cnntrain.annotations.TrainedArchitecture;

import java.util.*;

/**
 *
 */
public class ReinforcementConfigurationData extends ConfigurationData {
    private static final String AST_ENTRY_LEARNING_METHOD = "learning_method";
    private static final String AST_ENTRY_NUM_EPISODES = "num_episodes";
    private static final String AST_ENTRY_DISCOUNT_FACTOR = "discount_factor";
    private static final String AST_ENTRY_NUM_MAX_STEPS = "num_max_steps";
    private static final String AST_ENTRY_TARGET_SCORE = "target_score";
    private static final String AST_ENTRY_TRAINING_INTERVAL = "training_interval";
    private static final String AST_ENTRY_USE_FIX_TARGET_NETWORK = "use_fix_target_network";
    private static final String AST_ENTRY_TARGET_NETWORK_UPDATE_INTERVAL = "target_network_update_interval";
    private static final String AST_ENTRY_SNAPSHOT_INTERVAL = "snapshot_interval";
    private static final String AST_ENTRY_AGENT_NAME = "agent_name";
    private static final String AST_ENTRY_USE_DOUBLE_DQN = "use_double_dqn";
    private static final String AST_ENTRY_LOSS = "loss";
    private static final String AST_ENTRY_RL_ALGORITHM = "rl_algorithm";
    private static final String AST_ENTRY_REPLAY_MEMORY = "replay_memory";
    private static final String AST_ENTRY_STRATEGY = "strategy";
    private static final String AST_ENTRY_ENVIRONMENT = "environment";
    private static final String AST_ENTRY_START_TRAINING_AT = "start_training_at";
    private static final String AST_SOFT_TARGET_UPDATE_RATE = "soft_target_update_rate";
    private static final String AST_EVALUATION_SAMPLES = "evaluation_samples";

    private static final String ENVIRONMENT_PARAM_REWARD_TOPIC = "reward_topic";
    private static final String ENVIRONMENT_ROS = "ros_interface";
    private static final String ENVIRONMENT_GYM = "gym";

    private static final String STRATEGY_ORNSTEIN_UHLENBECK = "ornstein_uhlenbeck";

    public ReinforcementConfigurationData(ConfigurationSymbol configuration, String instanceName) {
        super(configuration, instanceName);
    }

    public Boolean isSupervisedLearning() {
        if (configurationContainsKey(AST_ENTRY_LEARNING_METHOD)) {
            return retrieveConfigurationEntryValueByKey(AST_ENTRY_LEARNING_METHOD)
                    .equals(LearningMethod.SUPERVISED);
        }
        return true;
    }

    public Boolean isReinforcementLearning() {
        return configurationContainsKey(AST_ENTRY_LEARNING_METHOD)
                && retrieveConfigurationEntryValueByKey(AST_ENTRY_LEARNING_METHOD).equals(LearningMethod.REINFORCEMENT);
    }

    public Integer getNumEpisodes() {
        return !configurationContainsKey(AST_ENTRY_NUM_EPISODES)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(AST_ENTRY_NUM_EPISODES);
    }

    public Double getDiscountFactor() {
        return !configurationContainsKey(AST_ENTRY_DISCOUNT_FACTOR)
                ? null : (Double)retrieveConfigurationEntryValueByKey(AST_ENTRY_DISCOUNT_FACTOR);
    }

    public Integer getNumMaxSteps() {
        return !configurationContainsKey(AST_ENTRY_NUM_MAX_STEPS)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(AST_ENTRY_NUM_MAX_STEPS);
    }

    public Double getTargetScore() {
        return !configurationContainsKey(AST_ENTRY_TARGET_SCORE)
                ? null : (Double)retrieveConfigurationEntryValueByKey(AST_ENTRY_TARGET_SCORE);
    }

    public Integer getTrainingInterval() {
        return !configurationContainsKey(AST_ENTRY_TRAINING_INTERVAL)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(AST_ENTRY_TRAINING_INTERVAL);
    }

    public Boolean getUseFixTargetNetwork() {
        return !configurationContainsKey(AST_ENTRY_USE_FIX_TARGET_NETWORK)
                ? null : (Boolean)retrieveConfigurationEntryValueByKey(AST_ENTRY_USE_FIX_TARGET_NETWORK);
    }

    public Integer getTargetNetworkUpdateInterval() {
        return !configurationContainsKey(AST_ENTRY_TARGET_NETWORK_UPDATE_INTERVAL)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(AST_ENTRY_TARGET_NETWORK_UPDATE_INTERVAL);
    }

    public Integer getSnapshotInterval() {
        return !configurationContainsKey(AST_ENTRY_SNAPSHOT_INTERVAL)
                ? null : (Integer)retrieveConfigurationEntryValueByKey(AST_ENTRY_SNAPSHOT_INTERVAL);
    }

    public String getAgentName() {
        return !configurationContainsKey(AST_ENTRY_AGENT_NAME)
                ? null : (String)retrieveConfigurationEntryValueByKey(AST_ENTRY_AGENT_NAME);
    }

    public Boolean getUseDoubleDqn() {
        return !configurationContainsKey(AST_ENTRY_USE_DOUBLE_DQN)
                ? null : (Boolean)retrieveConfigurationEntryValueByKey(AST_ENTRY_USE_DOUBLE_DQN);
    }

    public Double getSoftTargetUpdateRate() {
        return !configurationContainsKey(AST_SOFT_TARGET_UPDATE_RATE)
                ? null : (Double)retrieveConfigurationEntryValueByKey(AST_SOFT_TARGET_UPDATE_RATE);
    }

    public Integer getStartTrainingAt() {
        return !configurationContainsKey(AST_ENTRY_START_TRAINING_AT)
            ? null : (Integer)retrieveConfigurationEntryValueByKey(AST_ENTRY_START_TRAINING_AT);
    }

    public Integer getEvaluationSamples() {
        return !configurationContainsKey(AST_EVALUATION_SAMPLES)
            ? null : (Integer)retrieveConfigurationEntryValueByKey(AST_EVALUATION_SAMPLES);
    }



    public RLAlgorithm getRlAlgorithm() {
        if (!isReinforcementLearning()) {
            return null;
        }
        return !configurationContainsKey(AST_ENTRY_RL_ALGORITHM)
            ? RLAlgorithm.DQN : (RLAlgorithm)retrieveConfigurationEntryValueByKey(AST_ENTRY_RL_ALGORITHM);
    }

    public String getInputNameOfTrainedArchitecture() {
        if (!this.getConfiguration().getTrainedArchitecture().isPresent()) {
            throw new IllegalStateException("No trained architecture set");
        }
        TrainedArchitecture trainedArchitecture = getConfiguration().getTrainedArchitecture().get();
        // We allow only one input, the first one is the only input
        return trainedArchitecture.getInputs().get(0);
    }

    public String getOutputNameOfTrainedArchitecture() {
        if (!this.getConfiguration().getTrainedArchitecture().isPresent()) {
            throw new IllegalStateException("No trained architecture set");
        }
        TrainedArchitecture trainedArchitecture = getConfiguration().getTrainedArchitecture().get();
        // We allow only one output, the first one is the only output
        return trainedArchitecture.getOutputs().get(0);
    }

    public List<Integer> getStateDim() {
        if (!this.getConfiguration().getTrainedArchitecture().isPresent()) {
            return null;
        }
        final String inputName = getInputNameOfTrainedArchitecture();
        TrainedArchitecture trainedArchitecture = this.getConfiguration().getTrainedArchitecture().get();
        return trainedArchitecture.getDimensions().get(inputName);
    }

    public List<Integer> getActionDim() {
        if (!this.getConfiguration().getTrainedArchitecture().isPresent()) {
            return null;
        }
        final String outputName = getOutputNameOfTrainedArchitecture();
        TrainedArchitecture trainedArchitecture = this.getConfiguration().getTrainedArchitecture().get();
        return trainedArchitecture.getDimensions().get(outputName);
    }

    public String getLoss() {
        return !configurationContainsKey(AST_ENTRY_LOSS)
                ? null : retrieveConfigurationEntryValueByKey(AST_ENTRY_LOSS).toString();
    }

    public Map<String, Object> getReplayMemory() {
        return getMultiParamEntry(AST_ENTRY_REPLAY_MEMORY, "method");
    }

    public Map<String, Object> getStrategy() {
        assert isReinforcementLearning(): "Strategy parameter only for reinforcement learning but called in a " +
         " non reinforcement learning context";
        Map<String, Object> strategyParams = getMultiParamEntry(AST_ENTRY_STRATEGY, "method");
        if (strategyParams.get("method").equals(STRATEGY_ORNSTEIN_UHLENBECK)) {
            assert getConfiguration().getTrainedArchitecture().isPresent(): "Architecture not present," +
             " but reinforcement training";
            TrainedArchitecture trainedArchitecture = getConfiguration().getTrainedArchitecture().get();
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
        }
        return strategyParams;
    }

    public Map<String, Object> getEnvironment() {
        return getMultiParamEntry(AST_ENTRY_ENVIRONMENT, "environment");
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
        Map<String, Object> environmentParameters = getMultiParamEntry(AST_ENTRY_ENVIRONMENT, "environment");
        if (environmentParameters == null
            || !environmentParameters.containsKey("environment")) {
            return false;
        }
        return environmentParameters.containsKey(ENVIRONMENT_PARAM_REWARD_TOPIC);
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
