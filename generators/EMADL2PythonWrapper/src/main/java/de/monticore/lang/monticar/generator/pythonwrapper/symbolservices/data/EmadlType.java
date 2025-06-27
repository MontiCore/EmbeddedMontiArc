/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data;

/**
 *
 */
public enum EmadlType {
    Q ("Q"),
    Z ("Z"),
    B ("B");

    private final String name;

    EmadlType(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return this.name;
    }

    public static EmadlType fromString(final String t) {
        switch (t) {
            case "Q":
                return EmadlType.Q;
            case "Z":
                return EmadlType.Z;
            case "B":
                return EmadlType.B;
            default:
                throw new IllegalArgumentException("No EMADL type");
        }
    }
}
