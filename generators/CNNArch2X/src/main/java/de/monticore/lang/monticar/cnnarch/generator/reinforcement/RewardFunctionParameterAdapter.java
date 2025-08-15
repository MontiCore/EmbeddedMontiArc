/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator.reinforcement;

import de.monticore.lang.monticar.cnnarch.generator.annotations.RewardFunctionParameter;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 *
 */
public class RewardFunctionParameterAdapter implements RewardFunctionParameter {
    private final ComponentPortInformation adaptee;
    private String outputParameterName;
    private String inputStateParameterName;
    private String inputTerminalParameterName;

    public RewardFunctionParameterAdapter(final ComponentPortInformation componentPortInformation) {
        this.adaptee = componentPortInformation;
    }

    public List<String> getInputNames() {
        return this.adaptee.getAllInputs().stream()
                .map(PortVariable::getVariableName)
                .collect(Collectors.toList());
    }

    public List<String> getOutputNames() {
        return this.adaptee.getAllOutputs().stream()
                .map(PortVariable::getVariableName)
                .collect(Collectors.toList());
    }

    public Optional<String> getTypeOfInputPort(String portName) {
        return this.adaptee.getAllInputs().stream()
                .filter(port -> port.getVariableName().equals(portName))
                .map(port -> port.getEmadlType().toString())
                .findFirst();
    }

    public Optional<String> getTypeOfOutputPort(String portName) {
        return this.adaptee.getAllOutputs().stream()
                .filter(port -> port.getVariableName().equals(portName))
                .map(port -> port.getEmadlType().toString())
                .findFirst();
    }

    public Optional<List<Integer>> getInputPortDimensionOfPort(String portName) {
        return this.adaptee.getAllInputs().stream()
                .filter(port -> port.getVariableName().equals(portName))
                .map(PortVariable::getDimension)
                .findFirst();
    }

    public Optional<List<Integer>> getOutputPortDimensionOfPort(String portName) {
        return this.adaptee.getAllOutputs().stream()
                .filter(port -> port.getVariableName().equals(portName))
                .map(PortVariable::getDimension)
                .findFirst();
    }

    public Optional<String> getOutputParameterName() {
        if (this.outputParameterName == null) {
            if (this.getOutputNames().size() == 1) {
                this.outputParameterName = this.getOutputNames().get(0);
            } else {
                return Optional.empty();
            }
        }
        return Optional.of(this.outputParameterName);
    }


    private boolean isBooleanScalar(final PortVariable portVariable) {
        return portVariable.getEmadlType().equals(EmadlType.B)
                && portVariable.getDimension().size() == 1
                && portVariable.getDimension().get(0) == 1;
    }

    private boolean determineInputNames() {
        if (this.getInputNames().size() != 2) {
            return false;
        }
        Optional<String> terminalInput = this.adaptee.getAllInputs()
                .stream()
                .filter(this::isBooleanScalar)
                .map(PortVariable::getVariableName)
                .findFirst();

        if (terminalInput.isPresent()) {
            this.inputTerminalParameterName = terminalInput.get();
        } else {
            return false;
        }

        Optional<String> stateInput = this.adaptee.getAllInputs().stream()
                .filter(portVariable -> !portVariable.getVariableName().equals(this.inputTerminalParameterName))
                .filter(portVariable -> !isBooleanScalar(portVariable))
                .map(PortVariable::getVariableName)
                .findFirst();

        if (stateInput.isPresent()) {
            this.inputStateParameterName = stateInput.get();
        } else {
            this.inputTerminalParameterName = null;
            return false;
        }
        return true;
    }

    public Optional<String> getInputStateParameterName() {
        if (this.inputStateParameterName == null) {
            this.determineInputNames();
        }

        return Optional.ofNullable(this.inputStateParameterName);
    }

    public Optional<String> getInputTerminalParameter() {
        if (this.inputTerminalParameterName == null) {
            this.determineInputNames();
        }

        return Optional.ofNullable(this.inputTerminalParameterName);
    }
}