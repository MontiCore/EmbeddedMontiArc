/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class ComponentPortInformationBuilder {
    private String componentName;
    private final List<PortVariable> inputVariables;
    private final List<PortVariable> outputVariables;

    public ComponentPortInformationBuilder() {
        inputVariables = new ArrayList<>();
        outputVariables = new ArrayList<>();
    }

    public ComponentPortInformationBuilder withName(final String name) {
        this.componentName = name;
        return this;
    }

    public ComponentPortInformationBuilder withInputVariable(final String variableName) {
        inputVariables.add(
                PortVariable.primitiveVariableFrom(variableName, EmadlType.Q, PortDirection.INPUT)
        );
        return this;
    }

    public ComponentPortInformationBuilder withInputVariable(final PortVariable portVariable) {
        this.inputVariables.add(portVariable);
        return this;
    }

    public ComponentPortInformationBuilder withInputVariable(PortVariable... portVariables) {
        for (PortVariable v : portVariables) {
            withInputVariable(v);
        }
        return this;
    }

    public ComponentPortInformationBuilder withOutputVariable(final String variableName) {
        outputVariables.add(
                PortVariable.primitiveVariableFrom(variableName, EmadlType.Q, PortDirection.OUTPUT)
        );
        return this;
    }

    public ComponentPortInformationBuilder withOutputVariable(final PortVariable portVariable) {
        outputVariables.add(portVariable);
        return this;
    }

    public ComponentPortInformationBuilder withOutputVariable(PortVariable... portVariables) {
        for (PortVariable v : portVariables) {
            withOutputVariable(v);
        }
        return this;
    }

    public ComponentPortInformation build() {
        ComponentPortInformation wrapperIO = new ComponentPortInformation(this.componentName);

        wrapperIO.addAllInputs(this.inputVariables);
        wrapperIO.addAllOutputs(this.outputVariables);

        return wrapperIO;
    }
}
