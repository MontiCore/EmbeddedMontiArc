/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.can;

import de.rwth.montisim.simulation.eesimulator.bus.OperationMode;

public enum CANOperationMode implements OperationMode {
    //40m length
    HIGH_SPEED_CAN("HIGH_SPEED_CAN", 1),
    //100m length
    MEDIUM_SPEED_CAN("MEDIUM_SPEED_CAN", 0.5),
    //500m length
    LOW_SPEED_CAN("LOW_SPEED_CAN", 0.125);

    private final double dataRate;

    private final String name;

    private CANOperationMode(String name, double dataRate) {
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
