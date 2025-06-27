package de.monticore.lang.monticar.cnnarch.generator.training;

import java.util.Optional;

public class Hyperparameter {

    private String parameterName;
    private Object value;
    private Optional<Object> defaultValue;

    public Hyperparameter(String parameterName, Object value) {
        this.parameterName = parameterName;
        this.value = value;
    }

    public String getParameterName() {
        return parameterName;
    }

    public void setParameterName(String parameterName) {
        this.parameterName = parameterName;
    }

    public Object getValue() {
        return value;
    }

    public void setValue(Object value) {
        this.value = value;
    }

    public Optional<Object> getDefaultValue() {
        return defaultValue;
    }

    public void setDefaultValue(Optional<Object> defaultValue) {
        this.defaultValue = defaultValue;
    }
}