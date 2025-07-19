/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.flexray;

import de.rwth.montisim.simulation.eesimulator.bus.OperationMode;

public enum FlexRayOperationMode implements OperationMode {
    REDUNDANCY("REDUNDANCY", 10),
    MAX_DATA_RATE("MAX_DATA_RATE", 20),
    ;

    private final double dataRate;

    private final String name;

    private FlexRayOperationMode(String name, double dataRate) {
        this.dataRate = dataRate;
        this.name = name;
    }

    @Override
    public double getDataRate() {
        return dataRate;
    }

    @Override
    public double getBitErrorRate() {
        return 0;
    }

    @Override
    public String toString() {
        return name;
    }
}
