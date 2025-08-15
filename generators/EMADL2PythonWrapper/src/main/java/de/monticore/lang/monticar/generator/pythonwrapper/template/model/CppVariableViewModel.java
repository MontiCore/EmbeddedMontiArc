/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template.model;

import java.util.Objects;

/**
 *
 */
public class CppVariableViewModel {
    private final String type;
    private final String variableName;

    public CppVariableViewModel(String type, String variableName) {
        this.type = type;
        this.variableName = variableName;
    }

    public String getType() {
        return type;
    }

    public String getVariableName() {
        return variableName;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof CppVariableViewModel)) return false;
        CppVariableViewModel that = (CppVariableViewModel) o;
        return Objects.equals(type, that.type) &&
                Objects.equals(variableName, that.variableName);
    }

    @Override
    public int hashCode() {
        return Objects.hash(type, variableName);
    }

    @Override
    public String toString() {
        return "CppVariableViewModel{" +
                "type='" + type + '\'' +
                ", variableName='" + variableName + '\'' +
                '}';
    }
}
