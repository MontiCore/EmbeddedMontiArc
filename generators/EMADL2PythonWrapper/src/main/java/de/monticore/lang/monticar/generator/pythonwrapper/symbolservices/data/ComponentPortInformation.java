/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data;

import com.google.common.collect.ImmutableList;

import java.util.ArrayList;
import java.util.List;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;

/**
 *
 */
public class ComponentPortInformation {
    private final String componentName;

    private final List<PortVariable> inputs;
    private final List<PortVariable> outputs;

    public ComponentPortInformation(final String componentName) {
        this.componentName = componentName;
        this.inputs = new ArrayList<>();
        this.outputs = new ArrayList<>();
    }

    public String getComponentName() {
        return componentName;
    }

    public void addInput(final PortVariable input) {
        checkNotNull(input);
        checkArgument(input.isIncomingPort());
        this.inputs.add(input);
    }

    public void addOutput(final PortVariable output) {
        checkNotNull(output);
        checkArgument(!output.isIncomingPort());
        this.outputs.add(output);
    }

    public PortVariable getInput(final int index) {
        return this.inputs.get(index);
    }

    public PortVariable getOutput(final int index) {
        return this.outputs.get(index);
    }

    public List<PortVariable> getAllInputs() {
        return ImmutableList.copyOf(inputs);
    }

    public List<PortVariable> getAllOutputs() {
        return ImmutableList.copyOf(outputs);
    }

    public void addAllInputs(final List<PortVariable> inputs) {
        checkNotNull(inputs);
        checkArgument(inputs.stream().noneMatch(PortVariable::isOutgoingPort));
        this.inputs.addAll(inputs);
    }

    public void addAllOutputs(final List<PortVariable> outputs) {
        checkNotNull(outputs);
        checkArgument(outputs.stream().noneMatch(PortVariable::isIncomingPort));
        this.outputs.addAll(outputs);
    }
}
