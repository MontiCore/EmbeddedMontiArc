package de.monticore.lang.monticar.cnnarch.generator.util;

public enum PythonType {
    INTEGER("int"),
    STRING("str"),
    BOOLEAN("bool");

    private final String pythonType;

    PythonType(String pythonType) {
        this.pythonType = pythonType;
    }

    public String getType() {
        return pythonType;
    }
}
