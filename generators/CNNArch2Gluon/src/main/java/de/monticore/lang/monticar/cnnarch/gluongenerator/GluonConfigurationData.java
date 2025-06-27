/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.reinforcement.RewardFunctionParameterAdapter;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;

import java.util.*;

public class GluonConfigurationData extends ConfigurationData {

    public GluonConfigurationData(TrainingConfiguration trainingConfiguration,
                                  TrainingComponentsContainer trainingComponentsContainer, String instanceName) {
        super(trainingConfiguration, trainingComponentsContainer, instanceName, new CNNArch2GluonTrainParamSupportChecker());
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

    private Optional<RewardFunctionParameterAdapter> getRlRewardFunctionParameter() {
        Optional<RewardFunctionParameterAdapter> rewardFunctionParameter = trainingComponentsContainer.getRewardFunctionParameter();
        if (!hasRewardFunction() || !rewardFunctionParameter.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(rewardFunctionParameter.orElse(null));
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