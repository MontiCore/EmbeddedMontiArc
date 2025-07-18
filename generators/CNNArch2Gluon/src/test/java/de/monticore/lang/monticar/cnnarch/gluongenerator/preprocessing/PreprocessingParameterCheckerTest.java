/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator.preprocessing;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch.generator.preprocessing.PreprocessingComponentParameterAdapter;
import de.monticore.lang.monticar.cnnarch.generator.preprocessing.PreprocessingPortChecker;
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

public class PreprocessingParameterCheckerTest {
    private static final PortVariable INPUT1_PORT = PortVariable.primitiveVariableFrom("port1", EmadlType.Q,
            PortDirection.INPUT);
    private static final PortVariable INPUT2_PORT = PortVariable.primitiveVariableFrom("port2", EmadlType.B,
            PortDirection.INPUT);
    private static final PortVariable OUTPUT1_PORT = PortVariable.primitiveVariableFrom("port1_out", EmadlType.Q,
            PortDirection.OUTPUT);
    private static final PortVariable OUTPUT2_PORT = PortVariable.primitiveVariableFrom("port2_out", EmadlType.Q,
            PortDirection.OUTPUT);
    private static final PortVariable OUTPUT3_PORT = PortVariable.primitiveVariableFrom("port1", EmadlType.Q,
            PortDirection.OUTPUT);
    private static final PortVariable OUTPUT4_PORT = PortVariable.primitiveVariableFrom("port2", EmadlType.Q,
            PortDirection.OUTPUT);
    private static final String COMPONENT_NAME = "TestProcessingComponent";

    PreprocessingPortChecker uut = new PreprocessingPortChecker();

    @Before
    public void setup() {
       Log.getFindings().clear();
       Log.enableFailQuick(false);
    }

    @Test
    public void validProcessing() {
        // given
        PreprocessingComponentParameterAdapter adapter = getValidProcessingAdapter();

        // when
        uut.check(adapter);
        List<Finding> findings = Log.getFindings();

        assertEquals(0, findings.stream().filter(Finding::isError).count());
    }

    @Test
    public void invalidProcessingOutputNames() {
        // given
         PreprocessingComponentParameterAdapter adapter = getInvalidProcessingOutputNameAdapter();

        // when
        uut.check(adapter);
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().anyMatch(Finding::isError));
    }

    @Test
    public void invalidProcessingOutputNumber() {
        // given
        PreprocessingComponentParameterAdapter adapter = getInvalidProcessingOutputNumberAdapter();

        // when
        uut.check(adapter);
        List<Finding> findings = Log.getFindings();

        assertTrue(findings.stream().anyMatch(Finding::isError));
    }

    private RewardFunctionParameterAdapter getComponentWithNonScalarOutput() {
        ComponentPortInformation componentPortInformation = new ComponentPortInformation(COMPONENT_NAME);
        componentPortInformation.addAllInputs(getValidInputPortVariables());
        List<PortVariable> outputs = Lists.newArrayList(PortVariable.multidimensionalVariableFrom(
        "output", EmadlType.Q, PortDirection.OUTPUT, Lists.newArrayList(2,2)));
        componentPortInformation.addAllOutputs(outputs);
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

    private PreprocessingComponentParameterAdapter getValidProcessingAdapter() {
        ComponentPortInformation componentPortInformation
                = new ComponentPortInformation(COMPONENT_NAME);
        componentPortInformation.addAllInputs(getValidInputPortVariables());
        componentPortInformation.addAllOutputs(getValidOutputPorts());
        return new PreprocessingComponentParameterAdapter(componentPortInformation);
    }

    private PreprocessingComponentParameterAdapter getInvalidProcessingOutputNameAdapter() {
        ComponentPortInformation componentPortInformation
                = new ComponentPortInformation(COMPONENT_NAME);
        componentPortInformation.addAllInputs(getValidInputPortVariables());
        componentPortInformation.addAllOutputs(getInvalidOutputPorts());
        return new PreprocessingComponentParameterAdapter(componentPortInformation);
    }

    private PreprocessingComponentParameterAdapter getInvalidProcessingOutputNumberAdapter() {
        ComponentPortInformation componentPortInformation
                = new ComponentPortInformation(COMPONENT_NAME);
        componentPortInformation.addAllInputs(getValidInputPortVariables());
        componentPortInformation.addAllOutputs(getInvalidOutputPorts());
        return new PreprocessingComponentParameterAdapter(componentPortInformation);
    }

    private List<PortVariable> getValidOutputPorts() {
        return Lists.newArrayList(OUTPUT1_PORT, OUTPUT2_PORT);
    }

    private List<PortVariable> getInvalidOutputPorts() {
        return Lists.newArrayList(OUTPUT3_PORT, OUTPUT4_PORT);
    }

    private List<PortVariable> getInvalidOutputNumberPorts() {
        return Lists.newArrayList(OUTPUT1_PORT);
    }

    private List<PortVariable> getValidInputPortVariables() {
        return Lists.newArrayList(INPUT1_PORT, INPUT2_PORT);
    }
}