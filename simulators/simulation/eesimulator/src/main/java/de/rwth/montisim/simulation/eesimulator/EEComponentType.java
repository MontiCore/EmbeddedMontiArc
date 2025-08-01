/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator;

public enum EEComponentType {

    BUS("Bus"),
    SENSOR("Sensor"),
    BRIDGE("Bridge"),
    ACTUATOR("Actuator"),
    COMPUTER("Computer"),
    SERVICE("Service"),
    FUNCTION_BLOCK("FunctionBlock"),
    TEST_COMPONENT("TestComponent");

    private final String name;

    private EEComponentType(String name) {
        this.name = name;
    }


    public String toString() {
        return this.name;
    }

}
