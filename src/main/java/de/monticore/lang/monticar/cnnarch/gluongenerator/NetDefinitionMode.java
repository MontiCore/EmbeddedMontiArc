/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

/**
 *
 */
public enum NetDefinitionMode {
    ARCHITECTURE_DEFINITION,
    PREDICTION_PARAMETER,
    FORWARD_FUNCTION,
    PARAMETER_VALIDATION,
    ARTIFICIAL_ARCH_CLASS;

    public static NetDefinitionMode fromString(final String netDefinitionMode) {
        switch(netDefinitionMode) {
            case "ARCHITECTURE_DEFINITION":
                return ARCHITECTURE_DEFINITION;
            case "FORWARD_FUNCTION":
                return FORWARD_FUNCTION;
            case "PREDICTION_PARAMETER":
                return PREDICTION_PARAMETER;
            case "PARAMETER_VALIDATION":
                return PARAMETER_VALIDATION;
            case "ARTIFICICIAL_ARCH_CLASS":
                return ARTIFICIAL_ARCH_CLASS;
            default:
                throw new IllegalArgumentException("Unknown Net Definition Mode");
        }
    }
}
