/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator.reinforcement;

import de.monticore.lang.monticar.cnnarch.generator.annotations.NNArchitecture;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 *
 */
public class FunctionParameterChecker {
    private String inputStateParameterName;
    private String inputTerminalParameterName;
    private String outputParameterName;
    private RewardFunctionParameterAdapter rewardFunctionParameter;
    private NNArchitecture trainedArchitecture;

    public FunctionParameterChecker() {
    }

    public void check(final RewardFunctionParameterAdapter rewardFunctionParameter,
                      final NNArchitecture trainedArchitecture) {
        assert rewardFunctionParameter != null;
        assert trainedArchitecture != null;
        this.rewardFunctionParameter = rewardFunctionParameter;
        this.trainedArchitecture = trainedArchitecture;

        retrieveParameterNames();
        checkHasExactlyTwoInputs();
        checkHasExactlyOneOutput();
        checkHasStateAndTerminalInput();
        checkInputTerminalTypeAndDimension();
        checkStateDimensionEqualsTrainedArchitectureState();
        checkInputStateDimension();
        checkOutputDimension();
    }

    private void checkStateDimensionEqualsTrainedArchitectureState() {
        failIfConditionFails(stateInputOfNNArchitectureIsEqualToRewardState(),
                "State dimension of trained architecture is not equal to reward state dimensions.");
    }

    private boolean stateInputOfNNArchitectureIsEqualToRewardState() {
        assert trainedArchitecture.getInputs().size() == 1: "Trained architecture is not a policy network.";
        final String nnStateInputName = trainedArchitecture.getInputs().get(0);
        final List<Integer> dimensions = trainedArchitecture.getDimensions().get(nnStateInputName);

        return rewardFunctionParameter.getInputPortDimensionOfPort(inputStateParameterName).isPresent()
                && rewardFunctionParameter.getInputPortDimensionOfPort(inputStateParameterName).get().equals(dimensions);
    }

    private void checkHasExactlyTwoInputs() {
        failIfConditionFails(functionHasTwoInputs(), "Reward function must have exactly two input parameters: "
                + "One input needs to represents the environment's state and another input needs to be a "
                + "boolean value which expresses whether the environment's state is terminal or not");
    }

    private void checkHasExactlyOneOutput() {
        failIfConditionFails(functionHasOneOutput(), "Reward function must have exactly one output");
    }

    private void checkHasStateAndTerminalInput() {
        failIfConditionFails(inputParametersArePresent(),
                "Reward function must have exactly two input parameters: "
                        +"One input needs to represents the environment's state as a numerical scalar, vector or matrice, "
                        + "and another input needs to be a "
                        + "boolean value which expresses whether the environment's state is terminal or not");
    }

    private void checkInputStateDimension() {
        failIfConditionFails(isInputStateParameterDimensionBetweenOneAndThree(),
                "Reward function state parameter with dimension higher than three is not supported");
    }

    private void checkInputTerminalTypeAndDimension() {
        failIfConditionFails(inputTerminalIsBooleanScalar(), "Reward functions needs a terminal input which"
                + " is a boolean scalar");
    }

    private void checkOutputDimension() {
        failIfConditionFails(outputParameterIsScalar(), "Reward function output must be a scalar");
    }

    private void retrieveParameterNames() {
        this.inputStateParameterName = rewardFunctionParameter.getInputStateParameterName().orElse(null);
        this.inputTerminalParameterName = rewardFunctionParameter.getInputTerminalParameter().orElse(null);
        this.outputParameterName = rewardFunctionParameter.getOutputParameterName().orElse(null);
    }

    private boolean inputParametersArePresent() {
        return rewardFunctionParameter.getInputStateParameterName().isPresent()
                && rewardFunctionParameter.getInputTerminalParameter().isPresent();
    }

    private boolean functionHasOneOutput() {
        return rewardFunctionParameter.getOutputNames().size() == 1;
    }

    private boolean functionHasTwoInputs() {
        return rewardFunctionParameter.getInputNames().size() == 2;
    }

    private boolean isInputStateParameterDimensionBetweenOneAndThree() {
        return (rewardFunctionParameter.getInputPortDimensionOfPort(inputStateParameterName).isPresent())
                && (rewardFunctionParameter.getInputPortDimensionOfPort(inputStateParameterName).get().size() <= 3)
                && (rewardFunctionParameter.getInputPortDimensionOfPort(inputStateParameterName).get().size() > 0);
    }

    private boolean outputParameterIsScalar() {
        return (rewardFunctionParameter.getOutputPortDimensionOfPort(outputParameterName).isPresent())
                && (rewardFunctionParameter.getOutputPortDimensionOfPort(outputParameterName).get().size() == 1)
                && (rewardFunctionParameter.getOutputPortDimensionOfPort(outputParameterName).get().get(0) == 1);
    }

    private boolean inputTerminalIsBooleanScalar() {
        return (rewardFunctionParameter.getInputPortDimensionOfPort(inputTerminalParameterName).isPresent())
                && (rewardFunctionParameter.getTypeOfInputPort(inputTerminalParameterName).isPresent())
                && (rewardFunctionParameter.getInputPortDimensionOfPort(inputTerminalParameterName).get().size() == 1)
                && (rewardFunctionParameter.getInputPortDimensionOfPort(inputTerminalParameterName).get().get(0) == 1)
                && (rewardFunctionParameter.getTypeOfInputPort(inputTerminalParameterName).get().equals("B"));
    }

    private void failIfConditionFails(final boolean condition, final String message) {
        if (!condition) {
            fail(message);
        }
    }

    private void fail(final String message) {
        Log.error(message);
        //System.exit(-1);
    }
}