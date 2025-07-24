/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement;

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch.generator.annotations.NNArchitecture;
import de.monticore.lang.monticar.cnnarch.generator.reinforcement.FunctionParameterChecker;
import de.monticore.lang.monticar.cnnarch.generator.reinforcement.RewardFunctionParameterAdapter;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class FunctionParameterCheckerTest {
    private static final List<Integer> STATE_DIMENSIONS = Lists.newArrayList(3,2,4);
    private static final PortVariable STATE_PORT = PortVariable.multidimensionalVariableFrom("input1", EmadlType.Q,
            PortDirection.INPUT, STATE_DIMENSIONS);
    private static final PortVariable TERMINAL_PORT = PortVariable.primitiveVariableFrom("input2", EmadlType.B,
            PortDirection.INPUT);
    private static final PortVariable OUTPUT_PORT = PortVariable.primitiveVariableFrom("output1", EmadlType.Q,
            PortDirection.OUTPUT);
    private static final String COMPONENT_NAME = "TestRewardComponent";

    FunctionParameterChecker uut = new FunctionParameterChecker();

    @Before
    public void setup() {
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void validReward() {
        // given
        RewardFunctionParameterAdapter adapter = getValidRewardAdapter();

        // when
        uut.check(adapter, getValidTrainedArchitecture());
        List<Finding> findings = Log.getFindings();

        assertEquals(0, findings.stream().filter(Finding::isError).count());
    }

    @Test
    public void invalidRewardWithOneInput() {
        // given
        RewardFunctionParameterAdapter adapter = getComponentWithOneInput();

        // when
        uut.check(adapter, getValidTrainedArchitecture());
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().anyMatch(Finding::isError));
    }

    @Test
    public void invalidRewardWithTwoOutputs() {
        // given
        RewardFunctionParameterAdapter adapter = getComponentWithTwoOutputs();

        // when
        uut.check(adapter, getValidTrainedArchitecture());
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().anyMatch(Finding::isError));
    }

    @Test
    public void invalidRewardWithTerminalHasQType() {
        // given
        RewardFunctionParameterAdapter adapter = getComponentWithTwoQInputs();

        // when
        uut.check(adapter, getValidTrainedArchitecture());
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().anyMatch(Finding::isError));
    }

    @Test
    public void invalidRewardWithNonScalarOutput() {
        // given
        RewardFunctionParameterAdapter adapter = getComponentWithNonScalarOutput();

        // when
        uut.check(adapter, getValidTrainedArchitecture());
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().filter(Finding::isError).count() == 1);
    }

    @Test
    public void invalidRewardStateUnequalToTrainedArchitectureState1() {
        // given
        RewardFunctionParameterAdapter adapter = getValidRewardAdapter();
        NNArchitecture trainedArchitectureWithDifferenDimension = getTrainedArchitectureWithStateDimensions(
                Lists.newArrayList(6));

        // when
        uut.check(adapter, trainedArchitectureWithDifferenDimension);
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().filter(Finding::isError).count() == 1);
    }

    @Test
    public void invalidRewardStateUnequalToTrainedArchitectureState2() {
        // given
        RewardFunctionParameterAdapter adapter = getValidRewardAdapter();
        NNArchitecture trainedArchitectureWithDifferenDimension = getTrainedArchitectureWithStateDimensions(
                Lists.newArrayList(3, 8));

        // when
        uut.check(adapter, trainedArchitectureWithDifferenDimension);
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().filter(Finding::isError).count() == 1);
    }

    @Test
    public void invalidRewardStateUnequalToTrainedArchitectureState3() {
        // given
        RewardFunctionParameterAdapter adapter = getValidRewardAdapter();
        NNArchitecture trainedArchitectureWithDifferenDimension = getTrainedArchitectureWithStateDimensions(
                Lists.newArrayList(2,4,3));

        // when
        uut.check(adapter, trainedArchitectureWithDifferenDimension);
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().filter(Finding::isError).count() == 1);
    }

    private RewardFunctionParameterAdapter getComponentWithNonScalarOutput() {
        ComponentPortInformation componentPortInformation = new ComponentPortInformation(COMPONENT_NAME);
        componentPortInformation.addAllInputs(getValidInputPortVariables());
        List<PortVariable> outputs = Lists.newArrayList(PortVariable.multidimensionalVariableFrom(
                "output", EmadlType.Q, PortDirection.OUTPUT, Lists.newArrayList(2,2)));
        componentPortInformation.addAllOutputs(outputs);
        return new RewardFunctionParameterAdapter(componentPortInformation);
    }

    private RewardFunctionParameterAdapter getComponentWithTwoQInputs() {
        ComponentPortInformation componentPortInformation
                = new ComponentPortInformation(COMPONENT_NAME);
        List<PortVariable> inputs = Lists.newArrayList(STATE_PORT,
                PortVariable.multidimensionalVariableFrom("input2", EmadlType.Q, PortDirection.INPUT,
                        Lists.newArrayList(2,3,2)));
        componentPortInformation.addAllInputs(inputs);
        componentPortInformation.addAllOutputs(getValidOutputPorts());
        return new RewardFunctionParameterAdapter(componentPortInformation);
    }

    private RewardFunctionParameterAdapter getComponentWithTwoOutputs() {
        ComponentPortInformation componentPortInformation
                = new ComponentPortInformation(COMPONENT_NAME);
        componentPortInformation.addAllInputs(getValidInputPortVariables());
        List<PortVariable> outputs = getValidOutputPorts();
        outputs.add(PortVariable.primitiveVariableFrom("output2", EmadlType.B, PortDirection.OUTPUT));
        componentPortInformation.addAllOutputs(outputs);
        return new RewardFunctionParameterAdapter(componentPortInformation);
    }

    private RewardFunctionParameterAdapter getComponentWithOneInput() {
        ComponentPortInformation componentPortInformation
                = new ComponentPortInformation(COMPONENT_NAME);
        componentPortInformation.addAllInputs(Lists.newArrayList(STATE_PORT));
        componentPortInformation.addAllOutputs(getValidOutputPorts());
        return new RewardFunctionParameterAdapter(componentPortInformation);
    }

    private RewardFunctionParameterAdapter getValidRewardAdapter() {
        ComponentPortInformation componentPortInformation
                = new ComponentPortInformation(COMPONENT_NAME);
        componentPortInformation.addAllInputs(getValidInputPortVariables());
        componentPortInformation.addAllOutputs(getValidOutputPorts());
        return new RewardFunctionParameterAdapter(componentPortInformation);
    }

    private List<PortVariable> getValidOutputPorts() {
        return Lists.newArrayList(OUTPUT_PORT);
    }

    private List<PortVariable> getValidInputPortVariables() {
        return Lists.newArrayList(STATE_PORT, TERMINAL_PORT);
    }

    private NNArchitecture getValidTrainedArchitecture() {
        NNArchitecture nnArchitectureSymbol = mock(NNArchitecture.class);
        final String stateInputName = "stateInput";
        when(nnArchitectureSymbol.getInputs()).thenReturn(Lists.newArrayList(stateInputName));
        when(nnArchitectureSymbol.getDimensions()).thenReturn(ImmutableMap.<String, List<Integer>>builder()
                .put(stateInputName, STATE_DIMENSIONS)
                .build());
        return nnArchitectureSymbol;
    }

    private NNArchitecture getTrainedArchitectureWithStateDimensions(final List<Integer> dimensions) {
        NNArchitecture nnArchitectureSymbol = mock(NNArchitecture.class);
        final String stateInputName = "stateInput";
        when(nnArchitectureSymbol.getInputs()).thenReturn(Lists.newArrayList(stateInputName));
        when(nnArchitectureSymbol.getDimensions()).thenReturn(ImmutableMap.<String, List<Integer>>builder()
                .put(stateInputName, dimensions)
                .build());
        return nnArchitectureSymbol;
    }
}