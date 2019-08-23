/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

/**
 *
 */
public enum NetDefinitionMode {
    ARCHITECTURE_DEFINITION,
    FORWARD_FUNCTION,
    PYTHON_INLINE,
    CPP_INLINE;

    public static NetDefinitionMode fromString(final String netDefinitionMode) {
        switch(netDefinitionMode) {
            case "ARCHITECTURE_DEFINITION":
                return ARCHITECTURE_DEFINITION;
            case "FORWARD_FUNCTION":
                return FORWARD_FUNCTION;
            case "PYTHON_INLINE":
                return PYTHON_INLINE;
            case "CPP_INLINE":
                return CPP_INLINE;
            default:
                throw new IllegalArgumentException("Unknown Net Definition Mode");
        }
    }
}
