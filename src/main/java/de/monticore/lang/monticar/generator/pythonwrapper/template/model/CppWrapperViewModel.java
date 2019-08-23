/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template.model;

import java.util.List;
import java.util.Objects;

/**
 *
 */
public class CppWrapperViewModel {
    private final String wrapperClassName;
    private final String wrappedComponentName;
    private final String inputClassName;
    private final String outputClassName;
    private final List<CppVariableViewModel> inputVariables;
    private final List<CppVariableViewModel> outputVariables;

    public CppWrapperViewModel(String wrapperClassName,
                               String wrappedComponentName,
                               String inputClassName,
                               String outputClassName,
                               List<CppVariableViewModel> inputVariables,
                               List<CppVariableViewModel> outputVariables) {
        this.wrapperClassName = wrapperClassName;
        this.wrappedComponentName = wrappedComponentName;
        this.inputClassName = inputClassName;
        this.outputClassName = outputClassName;
        this.inputVariables = inputVariables;
        this.outputVariables = outputVariables;
    }

    public String getWrapperClassName() {
        return wrapperClassName;
    }

    public String getWrappedComponentName() {
        return wrappedComponentName;
    }

    public String getInputClassName() {
        return inputClassName;
    }

    public String getOutputClassName() {
        return outputClassName;
    }

    public List<CppVariableViewModel> getInputVariables() {
        return inputVariables;
    }

    public List<CppVariableViewModel> getOutputVariables() {
        return outputVariables;
    }

    public String getCapitalizedWrapperClassName() {
        return this.wrapperClassName.toUpperCase();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof CppWrapperViewModel)) return false;
        CppWrapperViewModel that = (CppWrapperViewModel) o;
        return Objects.equals(wrapperClassName, that.wrapperClassName) &&
                Objects.equals(wrappedComponentName, that.wrappedComponentName) &&
                Objects.equals(inputClassName, that.inputClassName) &&
                Objects.equals(outputClassName, that.outputClassName) &&
                Objects.equals(inputVariables, that.inputVariables) &&
                Objects.equals(outputVariables, that.outputVariables);
    }

    @Override
    public int hashCode() {
        return Objects.hash(wrapperClassName, wrappedComponentName, inputClassName, outputClassName, inputVariables, outputVariables);
    }

    @Override
    public String toString() {
        return "CppWrapperViewModel{" +
                "wrapperClassName='" + wrapperClassName + '\'' +
                ", wrappedComponentName='" + wrappedComponentName + '\'' +
                ", inputClassName='" + inputClassName + '\'' +
                ", outputClassName='" + outputClassName + '\'' +
                ", inputVariables=" + inputVariables +
                ", outputVariables=" + outputVariables +
                '}';
    }
}
