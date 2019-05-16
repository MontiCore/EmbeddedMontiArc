package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionParameterAdapter;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.ConfigurationData;
import de.monticore.lang.monticar.cnntrain._symboltable.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

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

    private static final String AST_ENTRY_REPLAY_MEMORY = "replay_memory";
    private static final String AST_ENTRY_ACTION_SELECTION = "action_selection";
    private static final String AST_ENTRY_ENVIRONMENT = "environment";

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

    public String getLoss() {
        return !configurationContainsKey(AST_ENTRY_LOSS)
                ? null : retrieveConfigurationEntryValueByKey(AST_ENTRY_LOSS).toString();
    }

    public Map<String, Object> getReplayMemory() {
        return getMultiParamEntry(AST_ENTRY_REPLAY_MEMORY, "method");
    }

    public Map<String, Object> getActionSelection() {
        return getMultiParamEntry(AST_ENTRY_ACTION_SELECTION, "method");
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
