package de.monticore.lang.monticar.cnnarch.pytorchgenerator;

public enum NetDefinitionMode {
    ARCHITECTURE_DEFINITION,
    FORWARD_FUNCTION;

    public static NetDefinitionMode fromString(final String netDefinitionMode) {
        switch(netDefinitionMode) {
            case "ARCHITECTURE_DEFINITION":
                return ARCHITECTURE_DEFINITION;
            case "FORWARD_FUNCTION":
                return FORWARD_FUNCTION;
            default:
                throw new IllegalArgumentException("Unknown Net Definition Mode");
        }
    }
}
